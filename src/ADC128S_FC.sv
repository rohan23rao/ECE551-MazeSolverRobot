`timescale 1ns/1ps
module ADC128S_FC(clk,rst_n,SS_n,SCLK,MISO,MOSI,IR_lft,IR_cntr,IR_rght,batt);
  //////////////////////////////////////////////////|
  // Model of a National Semi Conductor ADC128S    ||
  // 12-bit A2D converter.  NOTE: this model       ||
  // is used in the "MazeRunner" project and gives ||
  // warning for any channels other than 0,2,5,7   ||
  ///////////////////////////////////////////////////

  input clk,rst_n;		// clock and active low asynch reset
  input SS_n;			// active low slave select
  input SCLK;			// Serial clock
  input MOSI;			// serial data in from master
  input [11:0] IR_lft;	// reading you want for IR_lft
  input [11:0] IR_cntr;	// reading you want for IR_cntr
  input [11:0] IR_rght;	// reading you want for IR_rght
  input [11:0] batt;	// reading you want for batt
  
  output MISO;			// serial data out to master
  
  wire [15:0] A2D_data,cmd;
  wire rdy_rise;
	
  typedef enum reg {FIRST,SECOND} state_t;
  
  state_t state,nxt_state;
  
  ///////////////////////////////////////////////
  // Registers needed in design declared next //
  /////////////////////////////////////////////
  reg rdy_ff;				// used for edge detection on rdy
  reg [2:0] channel;		// pointer to last channel specified for A2D conversion to be performed on.
  reg [11:0] value;
  
  /////////////////////////////////////////////
  // SM outputs declared as type logic next //
  ///////////////////////////////////////////
  logic update_ch;

  ////////////////////////////////
  // Instantiate SPI interface //
  //////////////////////////////
  SPI_ADC128S iSPI(.clk(clk),.rst_n(rst_n),.SS_n(SS_n),.SCLK(SCLK),.MISO(MISO),
                 .MOSI(MOSI),.A2D_data(A2D_data),.cmd(cmd),.rdy(rdy));

	  
  //// channel pointer ////	  
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
	  channel <= 3'b000;
	else if (update_ch) begin
	  channel <= cmd[13:11];
	  if ((channel!=3'b000) && (channel!=3'b010) && (channel!=3'b101) && (channel!=3'b111))
	    $display("WARNING: Only channels 0,2,5,7 of A2D valid for this version of ADC128S\n");
	end
	
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
	  value <= 12'hxxx;
	else if (update_ch)
	  value <= (cmd[13:11]==3'b000) ? batt :
	           (cmd[13:11]==3'b010) ? IR_rght :
			   (cmd[13:11]==3'b101) ? IR_lft :
			   (cmd[13:11]==3'b111) ? IR_cntr :
			   12'hxxx;		// not a used channel
	  
  //// Infer state register next ////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
	  state <= FIRST;
	else
	  state <= nxt_state;
	  
  //// positive edge detection on rdy ////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
	  rdy_ff <= 1'b0;
	else
	  rdy_ff <= rdy;
  assign rdy_rise = rdy & ~rdy_ff;
  

  //////////////////////////////////////
  // Implement state tranisiton logic //
  /////////////////////////////////////
  always_comb
    begin
      //////////////////////
      // Default outputs //
      ////////////////////
      update_ch = 0;
      nxt_state = FIRST;	  

      case (state)
        FIRST : begin
          if (rdy_rise) begin
		    update_ch = 1;
            nxt_state = SECOND;
          end
        end
		SECOND : begin		
		  if (rdy_rise)
			nxt_state = FIRST;
		  else
		    nxt_state = SECOND;
		end
      endcase
    end
	
  assign A2D_data = {4'b0000,value};

endmodule  
  