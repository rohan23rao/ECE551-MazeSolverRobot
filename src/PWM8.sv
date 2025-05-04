`timescale 1ns/1ps
module PWM8(clk,rst_n,duty,PWM_sig);

  input clk,rst_n;		// clock and active low asynch reset
  input [7:0] duty;	// specifies duty cycle to motor drive
  output reg PWM_sig;
  
  wire set, reset;
  
  reg [7:0] cnt;
  

  
  ///////////////////////////
  // infer 8-bit counter //
  /////////////////////////
  always_ff @(posedge clk, negedge rst_n) 
    if (!rst_n)
	  cnt <= 8'h0;
	else
	  cnt <= cnt + 1;
	  
  assign set = ~|cnt;		// set at zero, but reset has priority
  assign reset = (cnt>=duty) ? 1'b1 : 1'b0;
  
  ////////////////////////////
  // infer PWM output flop //
  //////////////////////////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
	  PWM_sig <= 1'b0;
	else if (reset)
	  PWM_sig <= 1'b0;
	else if (set)
	  PWM_sig <= 1'b1;
  
endmodule
