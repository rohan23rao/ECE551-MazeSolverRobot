`timescale 1ns/1ps
module sensor_intf(clk,rst_n,IR_lft_en,IR_cntr_en,IR_rght_en,lft_IR,
                   rght_IR,IR_Dtrm,vbatt,lft_opn,rght_opn,strt_cal,
				   frwrd_opn,batt_low,A2D_SS_n,
				   A2D_SCLK,A2D_MOSI,A2D_MISO,LED);

  parameter FAST_SIM = 1'b1;
  parameter NOM_IR = 12'h900;		// we calibrate to acheive this + HUG
  
  input clk,rst_n;					// 50MHz clock and active low asynch reset
  input A2D_MISO;
  input strt_cal;					// get initial offset readings of IR_sensors
  output IR_lft_en;					// enable emitter (PWM'd)
  output IR_cntr_en;
  output IR_rght_en;
  output reg [11:0] lft_IR;				// A2D reading...to inert_intf for fusion
  output reg [11:0] rght_IR;			// A2D reading...to inert_intf for fusion
  output reg [8:0] IR_Dtrm;					// Derivative of IR readings (rght - lft)
  output [11:0] vbatt;					// A2D reading of battery voltage
  output lft_opn,frwrd_opn,rght_opn;	// opening in maze on left, center, right
  output batt_low;						// if two MSBs of batt not 11 then batt is low
  output A2D_SS_n;						// Serf select to A2D on DE0
  output A2D_MOSI, A2D_SCLK;			// MOSI and SCLK to A2D
  output [6:0] LED;

  typedef enum reg [2:0] {LFT_STTL,LFT_CPTR,CNTR_STTL,CNTR_CPTR,RGHT_STTL,
                          RGHT_CPTR,BATT_STTL,BATT_CPTR} state_t;
						  
  /////////////////////////////////////////////
  // states needed for calibration SM //
  /////////////////////////////////////
  typedef enum reg {IDLE,CAL} cal_state_t;
  
  state_t state,nxt_state;
  cal_state_t cal_state, nxt_cal_state;
  
  ///////////////////////////////
  // Declare internal signals //
  /////////////////////////////
  wire [11:0] res;
  wire cnv_cmplt;
  wire IR_duty;
  wire signed [8:0] lft_IR_Dtrm,rght_IR_Dtrm;
  wire signed [12:0] lft_IR_diff,rght_IR_diff;
  wire signed [9:0] IR_Dtrm_sum;
  wire sttl_ovr;				// settling time after IR enabled is over
  wire [13:0] vbatt_accum;		// used in formation of running average of vbatt
  wire [11:0] IR_err;			// used to set nominal IR reading to be IR_NOM + HUG
  
  ////////////////////////////////////
  // Declare state machine outputs //
  //////////////////////////////////
  logic strt_cnv;
  logic capture_batt,capture_rght,capture_lft,capture_cntr;
  logic [2:0] chnnl;
  logic [1:0] IR_sel;
  ////// SM outputs of cal SM ////////
  logic clr_avg_accums;
  logic accum_lft,accum_rght;
  logic ld_offs;

  ////////////////////////////////////////
  // declare needed internal registers //
  //////////////////////////////////////
  reg [11:0] cntr_IR;
  reg [15:0] tmr;			// timer to kick off round robins
  reg [11:0] lft_IR_prev1,rght_IR_prev1;	// used to develop Dterm
  reg [11:0] lft_IR_prev2,rght_IR_prev2;	// used to develop Dterm
  reg [11:0] lft_IR_prev3,rght_IR_prev3;	// used to develop Dterm
  reg [11:0] vbatt1,vbatt2,vbatt3,vbatt4;	// running average of 4 samples
  //// Registers needed for IR calibration below ////
  reg [11:0] lft_off_reg,rght_off_reg;		// hold offset calibration
  reg [14:0] lft_accum,rght_accum;			// registers to accumulate average of 8
  reg [3:0] smpl_cnt;

  localparam OPEN_THRES = 12'hD30;
  localparam OPEN_THRES_FRNT = 12'hDC0;
  localparam HUG = 12'h0E0;			// want nominal IR reading to be NOM_IR + 0xE0
									// biases it toward wall when flying with 1 IR
  
  ////////////////////////////////////////
  // Speed up simulation when FAST_SIM //
  //////////////////////////////////////
  generate if (FAST_SIM)
	assign sttl_ovr = &tmr[10:0];
  else
	assign sttl_ovr = &tmr;
  endgenerate
  
  ///////////////////////////////////////////////////////
  // Settling time is greater than 3 Tc of IR circuit //
  /////////////////////////////////////////////////////  
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
	  tmr <= 16'h0000;
	else
	  tmr <= tmr + 1;
	  
  ///// Flop to capture battery voltage //////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n) begin
      vbatt1 <= 12'hC00;
	  vbatt2 <= 12'hC00;
	  vbatt3 <= 12'hC00;
	  vbatt4 <= 12'hC00;
    end else if (capture_batt) begin
      vbatt1 <= res;
	  vbatt2 <= vbatt1;
	  vbatt3 <= vbatt2;
	  vbatt4 <= vbatt3;
	end
	
  //// compute average of last 4 battery voltages /////
  assign vbatt_accum = vbatt1 + vbatt2 + vbatt3 + vbatt4;
  assign vbatt = vbatt_accum[13:2];				// div 4 to get average
  
  assign LED = vbatt[11:5];
  
  assign batt_low = (vbatt[11:4]<8'hD9)? 1'b1 : 1'b0;	// if less than 5.36
    
  ///// Flop to capture center IR //////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
      cntr_IR <= 12'h0000;
    else if (capture_cntr)
      cntr_IR <= res;

  ///// Flop to capture right IR //////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n) begin
      rght_IR <= 12'h000;
	  rght_IR_prev1 <= 12'h000;
	  rght_IR_prev2 <= 12'h000;
	  rght_IR_prev3 <= 12'h000;
    end else if (capture_rght) begin
      rght_IR <= res - rght_off_reg;
	  rght_IR_prev1 <= rght_IR;
	  rght_IR_prev2 <= rght_IR_prev1;
	  rght_IR_prev3 <= rght_IR_prev2;
    end
	
  ///// Flop to capture left IR //////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n) begin
      lft_IR <= 12'h000;
	  lft_IR_prev1 <= 12'h000;
	  lft_IR_prev2 <= 12'h000;
	  lft_IR_prev3 <= 12'h000;
    end else if (capture_lft) begin
      lft_IR <= res - lft_off_reg;
	  lft_IR_prev1 <= lft_IR;
	  lft_IR_prev2 <= lft_IR_prev1;
	  lft_IR_prev3 <= lft_IR_prev2;
	end

  ////////////////////////////////////////////////////
  // Now form Dterm from difference in IR readings //
  //////////////////////////////////////////////////
  assign lft_IR_diff = {1'b0,lft_IR} - {1'b0,lft_IR_prev3};
  assign rght_IR_diff = {1'b0,rght_IR} - {1'b0,rght_IR_prev3};
  
  ///////////////////////////////////////////////////////
  // Now saturate to 9-bit cause don't need the width //
  /////////////////////////////////////////////////////
  assign lft_IR_Dtrm = (lft_IR_diff[12] & ~&lft_IR_diff[11:8]) ? 9'h100 :
                       (~lft_IR_diff[12] & |lft_IR_diff[11:8]) ? 9'h0FF :
					   lft_IR_diff[8:0];
  
  assign rght_IR_Dtrm = (rght_IR_diff[12] & ~&rght_IR_diff[11:8]) ? 9'h100 :
                        (~rght_IR_diff[12] & |rght_IR_diff[11:8]) ? 9'h0FF :
						rght_IR_diff[8:0]; 
						
  ///////////////////////////////////////////////////////////////
  // Calculate fusion correction factor based off IR readings //
  ///////////////////////////////////////////////////////////// 
  assign IR_Dtrm_sum = {lft_IR_Dtrm[8],lft_IR_Dtrm} - {rght_IR_Dtrm[8],rght_IR_Dtrm};

  /////////////////////////////////////////////////////
  // Need to pipeline (flop) IR_Dtrm for timing reasons //
  ///////////////////////////////////////////////////////
  always_ff @(posedge clk)
    IR_Dtrm = ((lft_opn) && (rght_opn)) ? 9'h000 :		// if no IR run with no fusion
              (lft_opn) ? ~{rght_IR_Dtrm[8],rght_IR_Dtrm[8:1]} :				// based on right if left is bad
	      (rght_opn) ? {lft_IR_Dtrm[8],lft_IR_Dtrm[8:1]} :				// based on lft if rght is bad
	      IR_Dtrm_sum[9:1];
						  
  ///////////////////////////
  // Implement state flop //
  /////////////////////////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
      state <= LFT_STTL;
    else
      state <= nxt_state;
	
  always_comb begin
    //////////////////////
    // Default Outputs //
    ////////////////////
    strt_cnv = 0;
    capture_batt = 0;
    capture_rght = 0;
    capture_lft = 0;
    capture_cntr = 0;
    chnnl = 3'bxxx;
    IR_sel = 2'b00;		// no IR enabled (when doing battery)
    nxt_state = state;
  
    case (state)
      LFT_STTL : begin
	    IR_sel = 2'b01;				// prepare for left
	    if (sttl_ovr) begin
	      chnnl = 3'b101;			// Conv left
	      strt_cnv = 1;
	      nxt_state = LFT_CPTR;
		end
	  end
	  LFT_CPTR : begin
	    IR_sel = 2'b01;				// still on left
		if (cnv_cmplt) begin
		  capture_lft = 1;
		  nxt_state = CNTR_STTL;
		end
	  end
      CNTR_STTL : begin
	    IR_sel = 2'b10;				// prepare for center
	    if (sttl_ovr) begin
	      chnnl = 3'b111;			// Conv center
	      strt_cnv = 1;
	      nxt_state = CNTR_CPTR;
		end
	  end	    
	  CNTR_CPTR : begin
	    IR_sel = 2'b10;				// still on center
		if (cnv_cmplt) begin
		  capture_cntr = 1;
		  nxt_state = RGHT_STTL;
		end
	  end
      RGHT_STTL : begin
	    IR_sel = 2'b11;				// prepare for right
	    if (sttl_ovr) begin
	      chnnl = 3'b010;			// Conv right
	      strt_cnv = 1;
	      nxt_state = RGHT_CPTR;
		end
	  end
	  RGHT_CPTR : begin
	    IR_sel = 2'b11;				// still on right
		if (cnv_cmplt) begin
		  capture_rght = 1;
		  nxt_state = BATT_STTL;
		end
	  end
	  BATT_STTL : begin
	    if (sttl_ovr) begin
	      chnnl = 3'b000;			// Conv batt
	      strt_cnv = 1;
	      nxt_state = BATT_CPTR;
		end
	  end
	  BATT_CPTR : begin
	    if (cnv_cmplt) begin
	      capture_batt = 1;
		  nxt_state = LFT_STTL;
	    end
	  end
    endcase
  end

  ///////////////////////////////////////////////////////////////
  // Implement averaging accumulators for lft/rght IR sensors //
  /////////////////////////////////////////////////////////////
  assign IR_err = res - NOM_IR - HUG;	// want IR reading to be NOM_IR + factor so will hug wall
  always_ff @(posedge clk)
    if (clr_avg_accums)
	  lft_accum <= 15'h0000;
	else if (accum_lft)
	  lft_accum <= lft_accum + {{3{IR_err[11]}},IR_err};

  always_ff @(posedge clk)
    if (clr_avg_accums)
	  rght_accum <= 15'h0000;
	else if (accum_rght)
	  rght_accum <= rght_accum + {{3{IR_err[11]}},IR_err};

  ////////////////////////////////////////////////
  // Implement sample counter for IR averaging //
  //////////////////////////////////////////////
  always_ff @(posedge clk)
    if (clr_avg_accums)
	  smpl_cnt <= 4'h0;
	else if (accum_rght)	// we know right happens 2nd so increment then
	  smpl_cnt <= smpl_cnt + 1;

  /////////////////////////////////////////////////////////
  // Implement offset registers for lft/rght IR sensors //
  ///////////////////////////////////////////////////////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n) begin
	  lft_off_reg <= 12'h000;
	  rght_off_reg <= 12'h000;
	end else if (clr_avg_accums) begin
	  lft_off_reg <= 12'h000;
	  rght_off_reg <= 12'h000;
	end else if (ld_offs) begin
	  lft_off_reg <= lft_accum[14:3];
	  rght_off_reg <= rght_accum[14:3];
	end
	  
  //////////////////////////////////////////
  // Implement state flop of cal machine //
  ////////////////////////////////////////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
      cal_state <= IDLE;
    else
      cal_state <= nxt_cal_state;
	  
  always_comb begin
    //////////////////////////////////
	// Default outputs & nxt_state //
	////////////////////////////////
    clr_avg_accums = 0;
    accum_lft = 0;
	accum_rght = 0;
	ld_offs = 0;
    nxt_cal_state = cal_state;
	
	case (cal_state)
	  IDLE : begin
	    if (strt_cal) begin
		  clr_avg_accums = 1;
		  nxt_cal_state = CAL;
		end
	  end
	  CAL : begin
	    accum_lft = capture_lft;
		accum_rght = capture_rght;
		if (smpl_cnt == 4'h8) begin
		  ld_offs = 1;
		  nxt_cal_state = IDLE;
		end
	  end
	endcase
  end

  /////////////////////////////////////////////////
  // Generate open signals based on IR readings //
  ///////////////////////////////////////////////
  assign lft_opn = (lft_IR>OPEN_THRES) ? 1'b1 : 1'b0;
  assign frwrd_opn = (cntr_IR>OPEN_THRES_FRNT) ? 1'b1 : 1'b0;
  assign rght_opn = (rght_IR>OPEN_THRES) ? 1'b1 : 1'b0;
 
  ///////////////////////////
  // Instantiate A2D_intf //
  /////////////////////////
  A2D_intf iA2D(.clk(clk),.rst_n(rst_n),.strt_cnv(strt_cnv),.cnv_cmplt(cnv_cmplt),
                .chnnl(chnnl),.res(res),.SS_n(A2D_SS_n),.SCLK(A2D_SCLK),.MOSI(A2D_MOSI),
  			    .MISO(A2D_MISO));

  ///////////////////////////////////////////
  // Instantiate PWM3 to drive IR enables //
  /////////////////////////////////////////
  PWM8 iDRV(.clk(clk),.rst_n(rst_n),.duty(8'hC0),.PWM_sig(IR_duty));
  
  assign IR_lft_en = (IR_sel==2'b01) ? IR_duty : 1'b0;
  assign IR_cntr_en = (IR_sel==2'b10) ? IR_duty : 1'b0;
  assign IR_rght_en = (IR_sel==2'b11) ? IR_duty : 1'b0;
   
endmodule
