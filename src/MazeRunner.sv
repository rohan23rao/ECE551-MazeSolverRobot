module MazeRunner(
  input clk, RST_n,						// 50MHz clock and asynch active low reset						// SPI input from A2D
  output INRT_SS_n,INRT_SCLK,INRT_MOSI,	// outputs of SPI to inertial interface
  input INRT_MISO,						// SPI input from gyro
  input INRT_INT,						// interrupt signals from gyro (new readings ready)
  output A2D_SS_n,A2D_SCLK,A2D_MOSI,	// SPI outputs to A2D
  input A2D_MISO,						// SPI input from A2D
  output lftPWM1,lftPWM2,				// left motor PWM controls
  output rghtPWM1,rghtPWM2,				// right motor PWM controls
  input RX,								// UART input from BLE module
  input hall_n,							// magnet detected (from hall sensor)
  output TX,							// UART output to BLE module
  output piezo,piezo_n,					// to Piezo buzzer (charge fanfare)
  output IR_lft_en,						// Enable Enable to left IR sensor
  output IR_cntr_en,					// Enable to center IR sensor
  output IR_rght_en,					// Enable to right IR sensor
  output [7:0] LED
);
 
  localparam FAST_SIM = 1;
  localparam NOM_IR = 12'h900;			// was 0x9E0

  ////////////////////////
  // Internals signals //
  //////////////////////
  wire rst_n;							// global synchronized reset
  wire strt_cal;						// initiate gyro heading calibration, also IR offset calibration
  logic cal_done;						// done with gyro heading calibration
  wire signed [11:0] lft_spd, rght_spd;	// signed motor controls
  wire signed [11:0] actl_hdng;			// actual heading as determined by inertial sensor
  wire signed [11:0] dsrd_hdng_cmd;		// desired heading from cmd_proc
  wire signed [11:0] dsrd_hdng_slv;		// desired heading from solver
  wire signed [11:0] dsrd_hdng;			// muxed desired heading to navigate
  wire signed [11:0] dsrd_hdng_adj;		// adjusted by IR readings
  wire strt_hdng_cmd,strt_hdng_slv;		// commands navigate unit to establish new heading (cmd_md or solver)
  wire strt_mv_cmd,strt_mv_slv;			// commands navigate unit to start new move (cmd_md or solver)
  wire strt_mv,strt_hdng;				// actual muxed start signals
  wire stp_lft_cmd,stp_rght_cmd;		// commands navigate unit in command mode
  wire stp_lft_slv,stp_rght_slv;		// commands navigate unit during solver mode
  wire stp_lft,stp_rght;				// actual stp signal lft/right to navigate unit
  wire [11:0] lft_IR,rght_IR;			// 12-bit unsigned IR readings for course corrections
  wire [8:0] IR_Dtrm; 					// Need to base fusion corrections on derivative as well
  wire [11:0] vbatt;					// battery voltage from A2D
  wire lft_opn,rght_opn,frwrd_opn;		// indicates an available direction in maze
  wire hdng_rdy;						// new heading reading is ready from inertial sensor
  wire moving;							// clear I in PID and don't integrate yaw if not moving
  wire en_fusion;						// only enable IR fusion correction when moving at decent speed
  wire at_hdng;							// dsrd_hdng & actl_hdng close enough
  wire send_resp;						// send either 0xA5 (done) or 0x5A (in progress)
  logic mv_cmplt;						// from navigate to either cmd_proc or maze_solve
  wire sol_cmplt;						// asserted when magnet found
  wire resp_sent;
  wire [10:0] frwrd_spd;				// forward speed
  wire [15:0] cmd;						// 16-bit cmd from bluetooth
  wire cmd_rdy;							// indicates command is ready 
  wire clr_cmd_rdy;						// asserted to mark command as processed
  wire batt_low;
  wire cmd_md;							// asserted by default, lowered when maze solve command issued
  
  assign sol_cmplt = ~hall_n;			// hall effect sensor is active low
  
  /////////////////////////////////////
  // Instantiate reset synchronizer //
  ///////////////////////////////////
  reset_synch iRST(.*);
 
  ///////////////////////////////////////////////////
  // UART_wrapper receives 16-bit command via BLE //
  /////////////////////////////////////////////////
  UART_Wrapper iWRAP(.clk(clk), .rst_n(rst_n), .RX(RX), .TX(TX), .resp(8'hA5), 
               .trmt(send_resp), .tx_done(resp_sent),
			   .cmd_rdy(cmd_rdy), .cmd(cmd), .clr_cmd_rdy(clr_cmd_rdy));
		
  ////////////////////////////////////
  // Instantiate command processor //
  //////////////////////////////////  
  cmd_proc iCMD(.clk(clk),.rst_n(rst_n),.cmd(cmd),.cmd_rdy(cmd_rdy),
           .clr_cmd_rdy(clr_cmd_rdy),.send_resp(send_resp),.strt_cal(strt_cal),
		   .cal_done(cal_done),.cmd_md(cmd_md),.dsrd_hdng(dsrd_hdng_cmd),
		   .strt_hdng(strt_hdng_cmd),.strt_mv(strt_mv_cmd),.stp_lft(stp_lft_cmd),
		   .stp_rght(stp_rght_cmd),.mv_cmplt(mv_cmplt),.sol_cmplt(sol_cmplt),
		   .in_cal(LED[0]));
		   
  maze_solve iSLV(.clk(clk),.rst_n(rst_n),.cmd0(cmd[0]),.cmd_md(cmd_md),
             .lft_opn(lft_opn),.rght_opn(rght_opn),.dsrd_hdng(dsrd_hdng_slv),
			 .strt_hdng(strt_hdng_slv),.strt_mv(strt_mv_slv),
			 .stp_lft(stp_lft_slv),.stp_rght(stp_rght_slv),.mv_cmplt(mv_cmplt)
			 ,.sol_cmplt(sol_cmplt));
				

  //////////////////////////////////////////////////////
  // Infer mux between cmd moves vs maze_solve moves //
  ////////////////////////////////////////////////////				  
  assign dsrd_hdng = (cmd_md) ? dsrd_hdng_cmd : dsrd_hdng_slv;
  assign strt_hdng = (cmd_md) ? strt_hdng_cmd : strt_hdng_slv;
  assign strt_mv = (cmd_md) ? strt_mv_cmd : strt_mv_slv;
  assign stp_lft = (cmd_md) ? stp_lft_cmd : stp_lft_slv;
  assign stp_rght = (cmd_md) ? stp_rght_cmd : stp_rght_slv;
 
  ///////////////////////////////////////////////////////
  // Instantiate navigate unit that controls movement //
  /////////////////////////////////////////////////////
  navigate #(FAST_SIM) iNAV(.clk(clk),.rst_n(rst_n),.strt_hdng(strt_hdng),.strt_mv(strt_mv),
                .stp_lft(stp_lft),.stp_rght(stp_rght),.mv_cmplt(mv_cmplt),.hdng_rdy(hdng_rdy),
				.moving(moving),.en_fusion(en_fusion),.at_hdng(at_hdng),.lft_opn(lft_opn),
				.rght_opn(rght_opn),.frwrd_opn(frwrd_opn),.frwrd_spd(frwrd_spd));
				

  assign LED[7:1] = 7'h00;	// only LSB of LED used and is "in_cal"

   
  /////////////////////////////////////
  // Instantiate inertial interface //
  ///////////////////////////////////
  inert_intf #(FAST_SIM) iNEMO(.clk(clk),.rst_n(rst_n),.strt_cal(strt_cal),
             .cal_done(cal_done),.heading(actl_hdng),.rdy(hdng_rdy),.IR_Dtrm(IR_Dtrm),  // IR_Dtrm
			 .SS_n(INRT_SS_n),.SCLK(INRT_SCLK),.MOSI(INRT_MOSI),.MISO(INRT_MISO),
			 .INT(INRT_INT),.moving(moving),.en_fusion(en_fusion));
  
  ////////////////////////////////////////////////////////////////////////////
  // Instantiate IR_Math which adjust desired heading based on IR readings //
  //////////////////////////////////////////////////////////////////////////
  IR_math #(NOM_IR) iIR_adj(.clk(clk),.rst_n(RST_n_n),.lft_opn(lft_opn),.rght_opn(rght_opn),.lft_IR(lft_IR),.rght_IR(rght_IR),
                            .IR_Dtrm(IR_Dtrm),.en_fusion(en_fusion),.dsrd_hdng(dsrd_hdng),
				            .dsrd_hdng_adj(dsrd_hdng_adj));//changed left and right speed
    
  /////////////////////////////////
  // Instantiate PID controller //
  ///////////////////////////////			 
  PID iCNTRL(.clk(clk),.rst_n(rst_n),.moving(moving),.dsrd_hdng(dsrd_hdng_adj),.actl_hdng(actl_hdng),
             .hdng_vld(hdng_rdy),.at_hdng(at_hdng),.frwrd_spd(frwrd_spd),.lft_spd(lft_spd),
			 .rght_spd(rght_spd));
					 
  
  ///////////////////////////////////
  // Instantiate motor PWM driver //
  /////////////////////////////////  
  MtrDrv iMTR(.clk(clk),.rst_n(rst_n),.lft_spd(lft_spd),.rght_spd(rght_spd),
              .vbatt(vbatt),.lftPWM1(lftPWM1),.lftPWM2(lftPWM2),
				  .rghtPWM1(rghtPWM1),.rghtPWM2(rghtPWM2));
			   
  ///////////////////////////////////////////////////////////////
  // Instantiate block to interface with IRs and Batt Voltage //
  /////////////////////////////////////////////////////////////				 
  sensor_intf #(FAST_SIM,NOM_IR) iIR(.clk(clk),.rst_n(rst_n),
                          .IR_lft_en(IR_lft_en),.strt_cal(strt_cal),
                          .IR_cntr_en(IR_cntr_en),.IR_rght_en(IR_rght_en),
						  .lft_IR(lft_IR),.rght_IR(rght_IR),.IR_Dtrm(IR_Dtrm),
						  .vbatt(vbatt),.lft_opn(lft_opn),.rght_opn(rght_opn),
						  .frwrd_opn(frwrd_opn),.batt_low(batt_low),
						  .A2D_SS_n(A2D_SS_n),.A2D_SCLK(A2D_SCLK),
						  .A2D_MOSI(A2D_MOSI),.A2D_MISO(A2D_MISO),.LED());
			  
  //////////////////////////////////////////////////////////////
  // Instantiate piezo_drv unit that plays "Charge! fanfare" //
  ////////////////////////////////////////////////////////////
  piezo_drv #(FAST_SIM) ICHRG(.clk(clk),.rst_n(rst_n),.batt_low(batt_low),.fanfare(sol_cmplt),
                           .piezo(piezo),.piezo_n(piezo_n));
  
	  
endmodule