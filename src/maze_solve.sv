module maze_solve(
clk, rst_n, cmd_md,cmd0,lft_opn,rght_opn,mv_cmplt,sol_cmplt,
strt_hdng,dsrd_hdng,strt_mv,stp_lft,stp_rght
);

input clk,rst_n,cmd_md,cmd0,lft_opn,rght_opn,mv_cmplt,sol_cmplt;
output logic strt_hdng,strt_mv,stp_lft,stp_rght;
output logic [11:0]dsrd_hdng;

  typedef enum reg[2:0] {IDLE,WAIT_MOVE,UPD_HDNG,ISS_HDNG,WAIT_HDNG,STRT_MOVE} state_t;
  state_t state,nstate;
  typedef enum reg[1:0] {NORTH,WEST,SOUTH,EAST} direction_t;
  direction_t cur_direct,nxt_direct;
  
  logic [11:0] nxt_dsrd_hdng;
  logic en_dsrd;
  //desired heading flop 
  always_ff @ (posedge clk, negedge rst_n)
	if(!rst_n)
		dsrd_hdng <= 0;
	else if(en_dsrd)
		dsrd_hdng <= nxt_dsrd_hdng;

  ////////////////////////
  // Infer State Flops //
  //////////////////////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
	   state <= IDLE;
	 else
	   state <= nstate;
//assign stp left and right 
assign stp_lft = (cmd0) ? 1'b1 : 
				1'b0;
assign stp_rght = (!cmd0) ? 1'b1 : 
				1'b0;

  //main state machine 
	always_comb begin 
		//default state machine outputs
		strt_hdng=0;
		strt_mv=0;
		nstate = state;
		nxt_dsrd_hdng=0;
		en_dsrd = 0;
		case(state)
		
			default: begin //IDLE state 
				if(!cmd_md)begin //active low 
				cur_direct = NORTH;
				nstate = WAIT_MOVE;
				strt_mv=1;
				end 
			end 
			
			WAIT_MOVE: begin 
				if(mv_cmplt)begin 
				nstate = UPD_HDNG;
				end 
			end 
			
			UPD_HDNG: begin //main logic state 
				//turn left when if lft affin and open or rght aff and only lft opn 
				if((stp_lft&lft_opn)|(~rght_opn&lft_opn))begin 
				//case determining what to update the heading to 
					case(cur_direct)
						NORTH: begin 
						nxt_dsrd_hdng = 12'h3FF;
						nxt_direct  = WEST;
						end 
						WEST: begin 
						nxt_dsrd_hdng = 12'h7FF;
						nxt_direct  = SOUTH;
						end 
						SOUTH: begin 
						nxt_dsrd_hdng = 12'hC00;
						nxt_direct  = EAST;
						end 
						EAST: begin 
						nxt_dsrd_hdng = 12'h000;
						nxt_direct  = NORTH;
						end 
					endcase
				end
				//turn right 
				else if((stp_rght&rght_opn)|(~lft_opn&rght_opn))begin 
				//case determining what to update the heading to 
					case(cur_direct)
						NORTH: begin 
						nxt_dsrd_hdng = 12'hC00;
						nxt_direct  = EAST;
						end 
						WEST: begin 
						nxt_dsrd_hdng = 12'h000;
						nxt_direct = NORTH;
						end 
						SOUTH: begin 
						nxt_dsrd_hdng = 12'h3FF;
						nxt_direct = WEST;
						end 
						EAST: begin 
						nxt_dsrd_hdng = 12'h7FF;
						nxt_direct = SOUTH;
						end 
					endcase
				end

				//what happens when neither are open if((~lft_opn)&&(~rght_opn))begin 
				else begin 
					case(cur_direct)
						NORTH: begin 
						nxt_dsrd_hdng = 12'h7FF;
						nxt_direct = SOUTH;
						end 
						WEST: begin 
						nxt_dsrd_hdng = 12'hC00;
						nxt_direct = EAST;
						end 
						SOUTH: begin 
						nxt_dsrd_hdng = 12'h000;
						nxt_direct  = NORTH;
						end 
						EAST: begin 
						nxt_dsrd_hdng = 12'h3FF;
						nxt_direct = WEST;
						end 
					endcase
				end	
				//move to next state now 
				nstate = ISS_HDNG;
				en_dsrd = 1; // enable the dsrd flop 
				if(sol_cmplt)
					nstate = IDLE;
			end 
			
			ISS_HDNG: begin 
				cur_direct = nxt_direct;
				strt_hdng=1;
				nstate = WAIT_HDNG;
			end 	
			
			WAIT_HDNG: begin 
				if(mv_cmplt)begin 
					nstate = STRT_MOVE;
					end 
			end 
			
			STRT_MOVE: begin 
				strt_mv = 1;
				nstate = WAIT_MOVE;
			end

		endcase
	end 

endmodule