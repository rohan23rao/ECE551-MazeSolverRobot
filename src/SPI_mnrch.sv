module SPI_mnrch(clk,rst_n,SS_n,SCLK,MOSI,MISO,wrt,wt_data,done,rd_data);

input clk,rst_n,wrt,MISO;
input [15:0]wt_data;
output logic SS_n,SCLK,MOSI,done;
output [15:0]rd_data;

logic [4:0]SCLK_div;
logic ld_SCLK,shft,smpl,MISO_smpl,done15,set_done,SCLK_fall_imm,init;
logic [15:0]shft_reg;
logic [3:0]bit_cntr;

logic [1:0]curr_state,nxt_state;

	typedef enum reg[1:0] {
	IDLE=2'b00,
	BEGI=2'b01,
	MAIN=2'b10,
	END=2'b11
	}states;

/*
SCLK_div flip flop 
*/

always_ff @(posedge clk)begin 
	if(ld_SCLK)begin 
	SCLK_div<=5'b10111;
	end 
	else begin 
	SCLK_div<=SCLK_div+1;
	end
end 

assign SCLK = SCLK_div[4];

//assign smpl 		
assign smpl = (~(SCLK_div^5'b01111)) ? 1'b1:
			1'b0;
//assign for fall immininent 
assign SCLK_fall_imm = &SCLK_div ? 1'b1:
						1'b0;


/*
shift register for the 16bit monarch 
*/
//MISO sample 
always_ff @(posedge clk)begin 
	if(smpl)begin 
	MISO_smpl<=MISO;
	end 
end 

//main shift reg 
always_ff @(posedge clk)begin 
	if(init)begin 
	shft_reg<=wt_data;
	end else if(~init&shft)begin
	shft_reg<={shft_reg[14:0],MISO_smpl};
	end
end 

//assign MOSI?
assign MOSI = shft_reg[15];

//ASSIGN read data 
assign rd_data = shft_reg;



/*
bit counter;
*/
always_ff @(posedge clk)begin 
	if(init)begin 
	bit_cntr<=4'b0;
	end else if (shft)begin 
	bit_cntr<=bit_cntr+1;
	end else begin 
	bit_cntr<=bit_cntr;
	end
end 

//assignment to see if done 
assign done15 = (&bit_cntr) ? 1'b1:
				1'b0;

/*
Main state machine 
*/

//main state machine flop 
always_ff @(posedge clk or negedge rst_n) begin 
	if(!rst_n) begin 
	  curr_state <= IDLE;
	end else begin 
      curr_state <= nxt_state;
	end 
end 

always_comb begin 
shft=0;
ld_SCLK=1;
init=0;
set_done=0;
nxt_state = curr_state;

case (curr_state)

	IDLE: begin 
		if(wrt)begin 
		init=1;
		ld_SCLK=0;
		nxt_state = BEGI;
		end 
	end 
	
	BEGI: begin
	ld_SCLK = 0;
		if(SCLK_fall_imm)begin 
		nxt_state = MAIN;
		end
	end 
	
	MAIN: begin 
	ld_SCLK = 0;
		shft = SCLK_fall_imm;
		if(done15) begin 
		nxt_state = END;
		end 
	end
	
	END: begin 
	ld_SCLK = 0;
		if(SCLK_fall_imm)begin 
		shft=1;
		set_done=1;
		ld_SCLK=1;
		nxt_state=IDLE;
		end 
	end 



endcase

end 

/*
2 set and reset flops 
*/

always_ff @(posedge clk, negedge rst_n)begin 
	if(!rst_n)begin 
	SS_n <= 1;
	end else if(init)begin
	SS_n <= 0;
	end else if(set_done)begin
	SS_n <= 1;
	end 
end 

always_ff @(posedge clk, negedge rst_n)begin 
	if(!rst_n)begin 
	done <= 1;
	end else if(init)begin
	done <= 0;
	end else if(set_done)begin
	done <= 1;
	end 
end 





endmodule