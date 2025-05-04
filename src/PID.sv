module PID(clk,rst_n,moving,dsrd_hdng,actl_hdng,hdng_vld,frwrd_spd,at_hdng,lft_spd,rght_spd);
input clk,rst_n;
input moving;
input signed [11:0]dsrd_hdng;
input signed [11:0]actl_hdng;
input hdng_vld;
input [10:0]frwrd_spd;
output at_hdng;
output signed [11:0]lft_spd;
output signed [11:0]rght_spd;

/*
*
P_term implementation 
*
*/
localparam P_COEFF = 4'h3;
logic signed [9:0]err_sat;
logic signed [13:0]P_term,P_term_pipe;
logic signed [11:0]error;
logic signed[11:0] actl_hdng_pipe;
/*
Implementation for the actl hdng and dsrd hdng sum 
*/
always_ff @(posedge clk)begin 
	actl_hdng_pipe <= actl_hdng;
end 


assign error = actl_hdng_pipe - dsrd_hdng; 

/*
*/
//saturate the input 
assign err_sat = (~error[11]&|error[10:9])?10'h1FF:
				   (error[11]&~&error[10:9])?10'h200:
				   error[9:0];
				   
//signed multipy 
assign P_term = err_sat*$signed(P_COEFF); 

always_ff @(posedge clk) begin
	P_term_pipe <= P_term;
end


/*
Err_sat < logic 
*/
logic [9:0]abs_err_sat;
assign abs_err_sat = (err_sat[9]==1) ? (-err_sat):
					 err_sat;
assign at_hdng = (abs_err_sat<10'd30) ? 1:
					0;
//what is at_hdng? 
/*
*
*I_term implementation
* 
*/

//define logics 
logic signed [15:0] integrator,nxt_integrator,ext,sum,sum_pipe;
logic ov,vld; 
logic signed [11:0]I_term,I_term_pipe;
//define flipflop 
always_ff@(posedge clk, negedge rst_n)
	if(!rst_n)
	   integrator<=16'h0000;
	else
		integrator<=nxt_integrator;


//sign extend err_sat 
assign ext = {{6{err_sat[9]}},err_sat[9:0]};
assign sum = ext + integrator;

always_ff @(posedge clk) begin
	sum_pipe <= sum;
end

//see if there is overflow 
assign ov = ((ext[15]~^integrator[15])&(ext[15]^sum_pipe[15])) ? 1:
			0;
//check if valid 
and(vld,~ov,hdng_vld);
			
//logic for the 2 mux 
assign nxt_integrator = (moving&vld) ? sum_pipe:
						(~moving) ? 16'h0000:
						integrator;
						
//shorten the output Iterm to 12 bits 
assign I_term = {integrator[15:4]};

always_ff @(posedge clk) begin
	I_term_pipe <= I_term;
end



/*
*
*D_term implementation 
*
*/
localparam [4:0]D_COEFF = 5'h0E;
logic signed [9:0]q1,q2;
logic signed [10:0]D_diff;
logic signed [7:0]satur; 
logic signed [12:0]D_term;
//flipflop 1
always_ff @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin 
		q1 <= 10'b0;
	end else if(hdng_vld) begin 
		q1 <= err_sat;
	end else begin 
		q1<=q1;
	end
end

//flipflop 2
always_ff @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin 
		q2 <= 10'b0;
	end else if(hdng_vld) begin 
		q2 <= q1;
	end else begin 
		q2<=q2;
	end
end

//minus q2 to prev err 
assign D_diff = err_sat - q2;
assign satur = (~D_diff[10]&|D_diff[9:7])?8'h7F:
				   (D_diff[10]&~&D_diff[9:7])?8'h80:
				   D_diff[7:0];
//signed multipy 
assign D_term = satur*$signed(D_COEFF); 

/*
Other logic 
*/
//sign extend all ans to 15 bit 
logic signed [14:0]SE_P_term;
logic signed [14:0]SE_I_term;
logic signed [14:0]SE_D_term;

assign SE_P_term = {P_term_pipe[13],P_term_pipe};
assign SE_I_term = {{3{I_term_pipe[11]}},I_term_pipe};
assign SE_D_term = {{2{D_term[12]}},D_term};

logic signed [11:0] sum_div8;
logic signed [14:0] PID_term;
assign PID_term = (SE_P_term + SE_I_term + SE_D_term) >> 3;
assign sum_div8 = PID_term[11:0]; // change to shift 

//final logics 
assign lft_spd = moving ? (sum_div8 + {0,frwrd_spd}):
						  12'h000;
assign rght_spd = moving ? ({0,frwrd_spd}-sum_div8):
						  12'h000;

endmodule 