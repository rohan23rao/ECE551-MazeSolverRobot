`timescale 1ns/1ps
module A2D_intf(clk,rst_n,strt_cnv,cnv_cmplt,chnnl,res,SS_n,SCLK,MOSI,MISO);

  input clk,rst_n;			// 50MHz clock and active low asynch reset
  input strt_cnv;			// initiates an A2D conversion
  input [2:0] chnnl;		// channel to perform conversion on
  input MISO;				// Serial input from A2D (Master In Slave Out)
  output [11:0] res;		// result of A2D conversion
  output logic cnv_cmplt;	// indicates full round robin conversions is complete
  output SS_n;				// active low SPI slave select to A2D
  output SCLK,MOSI;			// SPI master signals
  
  typedef enum reg[1:0] {IDLE,CMD,WAIT,RSP} state_t;
  state_t state, nxt_state;
  reg [7:0] wait_cnt;
  
  wire [15:0] rspns;		// data read from SPI interface.  Lower 12-bits form res (result of A2D conv)
  wire [15:0] cmd;
  wire done;

  logic wrt,clr_cnt;
  
  ///////////////////////////////////////////////////
  // Command to A2D is always 2'b00,chnnl,11'h000 //
  /////////////////////////////////////////////////
  assign cmd = {2'b00,chnnl,11'h000};		// command to A2D is simply address of channel to convert

  ////////////////////////////
  // Implement state flops //
  //////////////////////////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
	  state <= IDLE;
	else
	  state <= nxt_state;
	  
  /////////////////////////////
  // Implement wait counter //
  ///////////////////////////
  always_ff @(posedge clk)
    if (clr_cnt)
	  wait_cnt <= 8'h00;
	else
	  wait_cnt <= wait_cnt + 1;
	  
  always_comb begin
	wrt = 0;
	cnv_cmplt = 1'b0;
	clr_cnt = 0;
	nxt_state = IDLE;
	case (state)
	  IDLE:
	    if (strt_cnv) begin
		  nxt_state = CMD;
		  wrt = 1;
		end
	  CMD: begin
	    if (done) begin
		  clr_cnt = 1;
		  nxt_state = WAIT;
		end else
		  nxt_state = CMD;
	  end
	  WAIT: begin			// wait 1 extra clock between SPI transactions
		wrt = 1;
		nxt_state = RSP;
	  end
	  RSP: begin
	    if (done) begin
		  cnv_cmplt = 1;
		  nxt_state = IDLE;
		end else
		  nxt_state = RSP;
	  end
	endcase
  end
	  
  ///////////////////////////////////////////////
  // Instantiate SPI master for A2D interface //
  /////////////////////////////////////////////
  SPI_mnrch iSPI(.clk(clk),.rst_n(rst_n),.SS_n(SS_n),.SCLK(SCLK),.MISO(MISO),.MOSI(MOSI),.wrt(wrt),
                  .done(done),.rd_data(rspns),.wt_data(cmd));
  
  assign res = rspns[11:0];		// Give 1's complement as reading (invert for light line on dark background)
  
endmodule
  