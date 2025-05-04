`timescale 1ns/1ps
module MazeRunner_tb();

  //<< optional include or import >>
  
  reg clk,RST_n;
  reg send_cmd;					// assert to send command to MazeRunner_tb
  reg [15:0] cmd;				// 16-bit command to send
  reg [11:0] batt;				// battery voltage 0xDA0 is nominal
  
  logic cmd_sent;				
  logic resp_rdy;				// MazeRunner has sent a pos acknowledge
  logic [7:0] resp;				// resp byte from MazeRunner (hopefully 0xA5)
  logic hall_n;					// magnet found?
  
  /////////////////////////////////////////////////////////////////////////
  // Signals interconnecting MazeRunner to RunnerPhysics and RemoteComm //
  ///////////////////////////////////////////////////////////////////////
  wire TX_RX,RX_TX;
  wire INRT_SS_n,INRT_SCLK,INRT_MOSI,INRT_MISO,INRT_INT;
  wire lftPWM1,lftPWM2,rghtPWM1,rghtPWM2;
  wire A2D_SS_n,A2D_SCLK,A2D_MOSI,A2D_MISO;
  wire IR_lft_en,IR_cntr_en,IR_rght_en;  

  
  localparam FAST_SIM = 1'b0;

  //////////////////////
  // Instantiate DUT //
  ////////////////////
  MazeRunner iDUT(.clk(clk),.RST_n(RST_n),.INRT_SS_n(INRT_SS_n),.INRT_SCLK(INRT_SCLK),
                  .INRT_MOSI(INRT_MOSI),.INRT_MISO(INRT_MISO),.INRT_INT(INRT_INT),
				  .A2D_SS_n(A2D_SS_n),.A2D_SCLK(A2D_SCLK),.A2D_MOSI(A2D_MOSI),
				  .A2D_MISO(A2D_MISO),.lftPWM1(lftPWM1),.lftPWM2(lftPWM2),
				  .rghtPWM1(rghtPWM1),.rghtPWM2(rghtPWM2),.RX(RX_TX),.TX(TX_RX),
				  .hall_n(hall_n),.piezo(),.piezo_n(),.IR_lft_en(IR_lft_en),
				  .IR_rght_en(IR_rght_en),.IR_cntr_en(IR_cntr_en),.LED());
	
  ///////////////////////////////////////////////////////////////////////////////////////
  // Instantiate RemoteComm which models bluetooth module receiving & forwarding cmds //
  /////////////////////////////////////////////////////////////////////////////////////
  RemoteComm iCMD(.clk(clk), .rst_n(RST_n), .RX(TX_RX), .TX(RX_TX), .cmd(cmd), .send_cmd(send_cmd),
               .cmd_sent(cmd_sent), .resp_rdy(resp_rdy), .resp(resp));
			   
				  
  RunnerPhysics iPHYS(.clk(clk),.RST_n(RST_n),.SS_n(INRT_SS_n),.SCLK(INRT_SCLK),.MISO(INRT_MISO),
                      .MOSI(INRT_MOSI),.INT(INRT_INT),.lftPWM1(lftPWM1),.lftPWM2(lftPWM2),
					  .rghtPWM1(rghtPWM1),.rghtPWM2(rghtPWM2),
                     .IR_lft_en(IR_lft_en),.IR_cntr_en(IR_cntr_en),.IR_rght_en(IR_rght_en),
					 .A2D_SS_n(A2D_SS_n),.A2D_SCLK(A2D_SCLK),.A2D_MOSI(A2D_MOSI),
					 .A2D_MISO(A2D_MISO),.hall_n(hall_n),.batt(batt));
	
					 
  initial begin
	batt = 12'hDA0;  	// this is value to use with RunnerPhysics
  clk = 0;
  @(posedge clk);
  @(negedge clk);

  RST_n = 0;
  @(posedge clk);
  @(negedge clk);
  RST_n = 1;
  wait4sig(iPHYS.iNEMO.NEMO_setup, 1000000, clk, "NEMO_setup");
  
  // test calibration
	@(posedge clk);
  cmd = 16'h0000;
  send_cmd = 1'b1;
  @(posedge clk) send_cmd = 1'b0;
  //wait4sig(iDUT.cal_done, 1000000, clk, "cal_done");
  wait4sig(resp_rdy, 1000000, clk, "resp_rdy");
  if(resp !== 8'hA5) begin
    $display("ERROR: calibration failed");
    $stop();
  end

  manual_solve1();
  // auto_solve_tests();
  $stop();

  //print_location(iPHYS.xx, iPHYS.yy);
  
  @(posedge clk);
  cmd = 16'h2000; // heading north 
  send_cmd = 1'b1;
  @(posedge clk) send_cmd = 1'b0;
  wait4sig(resp_rdy, 100000000, clk, "resp_rdy");
  //rint_location(iPHYS.xx, iPHYS.yy);
  
  
  @(posedge clk);
  cmd = 16'h6000; // maze solve // try right affinity 
  send_cmd = 1'b1;
  @(posedge clk) send_cmd = 1'b0;
  wait4sig(resp_rdy, 100000000, clk, "resp_rdy");
  //print_location(iPHYS.xx, iPHYS.yy);

  repeat(10) @(posedge clk);
	$stop();

  end
  
  always
    #5 clk = ~clk;


  task automatic wait4sig(ref sig, input int clks2wait, ref clk, input string sig_name);
		fork
			begin: timeout
				repeat(clks2wait) @(posedge clk);
				$display("ERROR: timed out waiting for %s in wait4sig",sig_name);
				$stop();
			end
			begin
				@(posedge sig)
				disable timeout;
			end
		join
	endtask

  task automatic print_location(ref [14:0] xx, ref [14:0] yy);
		$display("The Robot is now at (%.2f, %.2f)", xx[14:12], yy[14:12]);
	endtask
  
  task automatic calibration();
    RST_n = 0;
    @(posedge clk);
    @(negedge clk);
    RST_n = 1;
    
    @(posedge clk);
    cmd = 16'h0000;
    send_cmd = 1'b1;
    @(posedge clk) send_cmd = 1'b0;
    //wait4sig(iDUT.cal_done, 1000000, clk, "cal_done");
    wait4sig(resp_rdy, 1000000, clk, "resp_rdy");
    if(resp !== 8'hA5) begin
      $display("ERROR: calibration failed");
      $stop();
    end

  endtask


  task automatic check_location(ref [14:0] xx, ref [14:0] yy, input [2:0] x, input [2:0] y);
    if (!((xx[14:12] == x) && (yy[14:12] == y))) begin
      $display("Location is not right");

    end
  
  endtask

  task automatic manual_solve1();

    @(posedge clk);    
    cmd = 16'h2000; // heading north 
    send_cmd = 1'b1;
    @(posedge clk) send_cmd = 1'b0;

    wait4sig(resp_rdy, 100000000, clk, "resp_rdy");
    print_location(iPHYS.xx, iPHYS.yy);
    check_location(iPHYS.xx, iPHYS.yy, 3'h2, 3'h0);

    @(posedge clk);
    cmd = 16'h4002; // move north 
    send_cmd = 1'b1;
    @(posedge clk) send_cmd = 1'b0;

    wait4sig(resp_rdy, 100000000, clk, "resp_rdy");
    print_location(iPHYS.xx, iPHYS.yy);
    check_location(iPHYS.xx, iPHYS.yy, 3'h2, 3'h1);

    @(posedge clk);
    cmd = 16'h23FF; // heading change west
    send_cmd = 1'b1;
    @(posedge clk) send_cmd = 1'b0;

    wait4sig(resp_rdy, 100000000, clk, "resp_rdy");
    print_location(iPHYS.xx, iPHYS.yy);
    check_location(iPHYS.xx, iPHYS.yy, 3'h2, 3'h1);


    @(posedge clk);
    cmd = 16'h4001; // move west
    send_cmd = 1'b1;
    @(posedge clk) send_cmd = 1'b0;

    wait4sig(resp_rdy, 100000000, clk, "resp_rdy");
    print_location(iPHYS.xx, iPHYS.yy);
    check_location(iPHYS.xx, iPHYS.yy, 3'h1, 3'h1);


    @(posedge clk);    
    cmd = 16'h2000; // heading north 
    send_cmd = 1'b1;
    @(posedge clk) send_cmd = 1'b0;

    wait4sig(resp_rdy, 100000000, clk, "resp_rdy");
    print_location(iPHYS.xx, iPHYS.yy);
    check_location(iPHYS.xx, iPHYS.yy, 3'h1, 3'h1);


    @(posedge clk);    
    cmd = 16'h4001; // move north 
    send_cmd = 1'b1;
    @(posedge clk) send_cmd = 1'b0;

    wait4sig(resp_rdy, 100000000, clk, "resp_rdy");
    print_location(iPHYS.xx, iPHYS.yy);
    check_location(iPHYS.xx, iPHYS.yy, 3'h1, 3'h2);


    @(posedge clk);
    cmd = 16'h2C00; // heading change east
    send_cmd = 1'b1;
    @(posedge clk) send_cmd = 1'b0;

    wait4sig(resp_rdy, 100000000, clk, "resp_rdy");
    print_location(iPHYS.xx, iPHYS.yy);
    check_location(iPHYS.xx, iPHYS.yy, 3'h1, 3'h2);

    @(posedge clk);    
    cmd = 16'h4001; // move east
    send_cmd = 1'b1;
    @(posedge clk) send_cmd = 1'b0;

    wait4sig(resp_rdy, 100000000, clk, "resp_rdy");
    print_location(iPHYS.xx, iPHYS.yy);
    check_location(iPHYS.xx, iPHYS.yy, 3'h3, 3'h2);


    @(posedge clk);    
    cmd = 16'h2000; // heading north 
    send_cmd = 1'b1;
    @(posedge clk) send_cmd = 1'b0;

    wait4sig(resp_rdy, 100000000, clk, "resp_rdy");
    print_location(iPHYS.xx, iPHYS.yy);
    check_location(iPHYS.xx, iPHYS.yy, 3'h3, 3'h2);


    @(posedge clk);
    cmd = 16'h4002; // move north 
    send_cmd = 1'b1;
    @(posedge clk) send_cmd = 1'b0;
    
    wait4sig(resp_rdy, 100000000, clk, "resp_rdy");
    print_location(iPHYS.xx, iPHYS.yy);
    check_location(iPHYS.xx, iPHYS.yy, 3'h3, 3'h3);


  endtask

  task automatic auto_solve_tests();
    // calibration();
  
    @(posedge clk);
    cmd = 16'h6000; // maze solve // try right affinity 
    send_cmd = 1'b1;
    @(posedge clk) send_cmd = 1'b0;
    wait4sig(resp_rdy, 100000000, clk, "resp_rdy");
    
    print_location(iPHYS.xx, iPHYS.yy); // Should be (2, 1)


  endtask


endmodule


// if (iPHYS.robot_heading === )

//     if (iPHYS.xx === 3, iPHYS.yy === 3) begin
//       $display();
//     end

//     if (iPHYS.omega_lft === ) begin

//     end