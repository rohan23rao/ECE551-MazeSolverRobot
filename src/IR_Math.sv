module IR_math(clk, rst_n, lft_opn, rght_opn, lft_IR, rght_IR, IR_Dtrm, en_fusion, dsrd_hdng, dsrd_hdng_adj);
input logic clk, rst_n;
input lft_opn, rght_opn, en_fusion;
input [11:0] lft_IR, rght_IR;
input signed [11:0] dsrd_hdng;
input signed [8:0] IR_Dtrm;
output signed [11:0] dsrd_hdng_adj;

parameter NOM_IR = 12'h900;

logic signed [11:0] IR_P_adj, IR_P_adj_pipe;

wire signed [12:0] IR_diff, IR_adj;

assign IR_diff = {1'b0, lft_IR} - {1'b0, rght_IR};

assign IR_P_adj = ((lft_opn) && (rght_opn)) ? 12'h000 :
				  (lft_opn) ? NOM_IR - rght_IR :
				  (rght_opn) ? lft_IR - NOM_IR :
				  IR_diff[12:1];

always @(posedge clk, negedge rst_n) begin
	if (!rst_n) begin
		IR_P_adj_pipe <= 0;
	end
	else
		IR_P_adj_pipe <= IR_P_adj;
end

assign IR_adj = {{6{IR_P_adj_pipe[11]}}, IR_P_adj_pipe[11:5]} + {{2{IR_Dtrm[8]}}, IR_Dtrm, 2'b0};

// always @(posedge clk, negedge rst_n) begin
// 	IR_adj2 <= IR_P_adj;
// end

assign dsrd_hdng_adj = (en_fusion) ? IR_adj[12:1] + dsrd_hdng	:
					   dsrd_hdng;
				  

endmodule