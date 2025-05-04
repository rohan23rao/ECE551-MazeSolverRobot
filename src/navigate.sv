module navigate(clk,rst_n,strt_hdng,strt_mv,stp_lft,stp_rght,mv_cmplt,hdng_rdy,moving,
                en_fusion,at_hdng,lft_opn,rght_opn,frwrd_opn,frwrd_spd);
				
  parameter FAST_SIM = 1;		// speeds up incrementing of frwrd register for faster simulation
				
  input clk,rst_n;					// 50MHz clock and asynch active low reset
  input strt_hdng;					// indicates should start a new heading
  input strt_mv;					// indicates should start a new forward move
  input stp_lft;					// indicates should stop at first left opening
  input stp_rght;					// indicates should stop at first right opening
  input hdng_rdy;					// new heading reading ready....used to pace frwrd_spd increments
  output logic mv_cmplt;			// asserted when heading or forward move complete
  output logic moving;				// enables integration in PID and in inertial_integrator
  output en_fusion;					// Only enable fusion (IR reading affect on nav) when moving forward at decent speed.
  input at_hdng;					// from PID, indicates heading close enough to consider heading complete.
  input lft_opn,rght_opn,frwrd_opn;	// from IR sensors, indicates available direction.  Might stop at rise of lft/rght
  output reg [10:0] frwrd_spd;		// unsigned forward speed setting to PID
  
  logic lft_opn_rise, rght_opn_rise;
  logic lq1,rq1;
  logic dec_frwrd,dec_frwrd_fast,inc_frwrd,init_frwrd;
  logic [5:0]frwrd_inc;
  logic [2:0]curr_state,nxt_state;
  
	typedef enum {
	IDLE = 3'b000,
	HDC = 3'b001,
	DECF = 3'b010,
	DEC = 3'b011,
	MV = 3'b100
	}states;

  
  localparam MAX_FRWRD = 11'h2A0;		// max forward speed
  localparam MIN_FRWRD = 11'h0D0;		// minimum duty at which wheels will turn
  ////////////////////////////////
  // Now form forward register //
  //////////////////////////////
  always_ff @(posedge clk, negedge rst_n) begin 
    if (!rst_n)
	  frwrd_spd <= 11'h000;
	else if (init_frwrd)		// assert this signal when leaving IDLE due to strt_mv
	  frwrd_spd <= MIN_FRWRD;									// min speed to get motors moving
	else if (hdng_rdy && inc_frwrd && (frwrd_spd<MAX_FRWRD))	// max out at 2A0
	  frwrd_spd <= frwrd_spd + {5'h00,frwrd_inc};				// always accel at 1x frwrd_inc
	else if (hdng_rdy && (frwrd_spd>11'h000) && (dec_frwrd | dec_frwrd_fast))
	  frwrd_spd <= ((dec_frwrd_fast) && (frwrd_spd>{2'h0,frwrd_inc,3'b000})) ? frwrd_spd - {2'h0,frwrd_inc,3'b000} : // 8x accel rate
                    (dec_frwrd_fast) ? 11'h000 :	  // if non zero but smaller than dec amnt set to zero.
	                (frwrd_spd>{4'h0,frwrd_inc,1'b0}) ? frwrd_spd - {4'h0,frwrd_inc,1'b0} : // slow down at 2x accel rate
					11'h000;
					
end

  ////////////////////////////////
  // Make edge detect          //
  //////////////////////////////
  always_ff @(posedge clk, negedge rst_n) begin 
	if(!rst_n)
		lq1<=0;
	else 
		lq1<=lft_opn;
  end 
 
  //check if lq1 is low and lft_opn is high 
  assign lft_opn_rise = (lft_opn&~lq1) ? 1:
						0;
						
  always_ff @(posedge clk, negedge rst_n) begin 
	if(!rst_n)
		rq1<=0;
	else 
		rq1<=rght_opn;
  end 
 
  //check if rq1 is low and rght_opn is high 
  assign rght_opn_rise = (rght_opn&~rq1) ? 1:
						0;

  //////////////////////////////////////
  // frwrd+inc depending upon FAST_SIM//
  //////////////////////////////////////
  
  generate if(FAST_SIM) begin 
	assign frwrd_inc = 6'h18;
	end else begin 
	assign frwrd_inc = 6'h02;
	end 
  endgenerate

  /////////////////////////////////////////
  // The function that controls en_fusion//
  /////////////////////////////////////////
  assign en_fusion = (frwrd_spd>(0.5*MAX_FRWRD)) ? 1:
					0;
  
  /////////////
  // The SM //
  ///////////
  
//main state machine flop 
always_ff @(posedge clk or negedge rst_n) begin 
	if(!rst_n) begin 
	  curr_state <= IDLE;
	end else begin 
      curr_state <= nxt_state;
	end 
end 

always_comb begin 
	//initialize stuff 
	dec_frwrd=0;
	dec_frwrd_fast=0;
	moving=0;
	mv_cmplt=0;
	inc_frwrd=0;
	nxt_state=curr_state;
	case(curr_state)
	
		IDLE: begin 
		//maybe moving should be 0?
		moving=0;
			if(strt_hdng)begin 
			nxt_state=HDC;
			end 
			if(strt_mv)begin 
			nxt_state=MV;
			init_frwrd=1;
			end
		end 
		
		HDC: begin 
		moving=1;
			if(at_hdng)begin 
			nxt_state=IDLE;
			mv_cmplt=1;
			end 
		end 
		
		DECF: begin 
		moving=1;
		dec_frwrd_fast=1;
			if(frwrd_spd==0)begin
			nxt_state=IDLE;
			mv_cmplt=1;
			end 
		end
		
		DEC: begin 
		moving=1;
		dec_frwrd=1;
			if(frwrd_spd==0)begin
			nxt_state=IDLE;
			mv_cmplt=1;
			end 	
		end 
		
		MV: begin 
		init_frwrd=0;
		moving=1;
		inc_frwrd=1;
			if(lft_opn_rise&&stp_lft)begin 
			nxt_state=DEC;
			end 
			else if(rght_opn_rise&&stp_rght)begin 
			nxt_state=DEC;
			end 
			else if(~frwrd_opn) begin 
			nxt_state=DECF;
			end
		end 
		
    endcase

end 



  
  



endmodule
  