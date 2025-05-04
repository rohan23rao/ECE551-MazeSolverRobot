module cmd_proc (cmd,cmd_rdy,clr_cmd_rdy,send_resp,strt_cal,
cal_done,in_cal,sol_cmplt,strt_hdng,strt_mv,stp_lft,stp_rght,
dsrd_hdng,mv_cmplt,cmd_md,clk,rst_n);

input [15:0]cmd;
input cmd_rdy,cal_done,sol_cmplt,mv_cmplt;
output logic clr_cmd_rdy, send_resp,strt_cal,in_cal,strt_hdng,strt_mv,stp_lft,stp_rght,cmd_md;
output logic [11:0]dsrd_hdng;
input clk,rst_n; //idk 

logic [2:0]op_code;
logic en_l,en_r;
logic en_dsrd_hdng;
  ////////////////////
  // Define States //
  //////////////////
  typedef enum reg[2:0] {IDLE,CALI,HDNG,MOVE,SOLV} state_t;
  state_t state,nstate;
  
  ////////////////////////
  // Infer State Flops //
  //////////////////////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
	   state <= IDLE;
	 else
	   state <= nstate;
  
	//capture flops for stp lft or rght
    always_ff @(posedge clk)
		if(en_l)
		stp_lft <= cmd[1];
  
    always_ff @(posedge clk)
		if(en_r)
		stp_rght <= cmd[0];
	
	//capture hdng 
	always_ff @(posedge clk)
	if(en_dsrd_hdng)
		dsrd_hdng <= cmd[11:0];
  

  assign op_code = cmd[15:13];
  
  always_comb begin 
	//initialize state machine output 
  nstate = state;
  cmd_md=1;
  clr_cmd_rdy=0;
  strt_cal=0;
  strt_hdng=0;
  en_dsrd_hdng=0;
  strt_mv=0;
  send_resp=0;
  	en_l=0;
	en_r=0;
  
  case(state)
  
	default: begin //This is the IDLE state 
		if(cmd_rdy)begin 
			case(op_code)
			
			3'b000 : begin 
			nstate = CALI;
			clr_cmd_rdy=1;
			strt_cal=1;
			end 
			
			3'b001 : begin 
			nstate = HDNG;
			clr_cmd_rdy=1;
			strt_hdng=1;
			en_dsrd_hdng=1;
			end 
			
			3'b010 : begin 
			nstate = MOVE;
			clr_cmd_rdy=1;
			strt_mv=1;
			en_l=1;
			en_r=1;
			end 
			
			3'b011 : begin 
			nstate = SOLV;
			clr_cmd_rdy=1;
			end 
			
			default : begin 
			nstate = IDLE;
			clr_cmd_rdy=1;
			//for future expansion 
			end 
			
			endcase
		end 		
	end 
  
  CALI: begin 
	in_cal=1;
	if(cal_done)begin 
	nstate = IDLE;
	send_resp=1;
	end 
  end 
  
  HDNG: begin 
  en_dsrd_hdng=0;
	if(mv_cmplt)begin
	nstate = IDLE;
	send_resp=1;
	end 
  end 
  
  MOVE: begin 
	if(mv_cmplt)begin
	nstate = IDLE;
	send_resp=1;
	end 
  end 
  
  SOLV: begin 
	cmd_md=0;
	if(sol_cmplt)begin 
	nstate = IDLE;
	send_resp=1;
	end 
  end 
  

  
  endcase
  
  end
  
  endmodule