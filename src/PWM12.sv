`timescale 1ns/1ps
module PWM12
(
input clk,rst_n,
input [11:0] duty,    //unsigned duty cycle
output logic PWM1, PWM2  // glitch free PWM signals with non_overlap
);

localparam NONOVERLAP = 12'h02C; // pre-defined localparam
logic [11:0] cnt;   // 12 bit counter for PWM generation
logic cnt_grt_sum;  // signal to used to tell that duty cycle is greater than duty+NONOVERLAP so set PWM2
logic rst_PWM2;     // signal used to reset PWM2 determined by &cnt
logic cnt_grt_param; // signal to inform cnt is larger than NONOVERLAP so we set PWM1
logic rst_PWM1;     // signal used in S/R flop to reset PWM1 determined by cnt >= duty


//////////////////////////////////////////////////////////
// combinational logic to set four signals for S/R flops//
//////////////////////////////////////////////////////////
assign rst_PWM2 = (&cnt)? 1'b1:1'b0;
assign cnt_grt_sum = (cnt>=(duty+NONOVERLAP))? 1'b1:1'b0;
assign cnt_grt_param = (cnt>= NONOVERLAP)? 1'b1:1'b0;
assign	rst_PWM1 = (cnt>= duty)? 1'b1:1'b0;

//////////////////////////////////////////
// Two S/R flops to determine PWM output//
//////////////////////////////////////////
always @ (posedge clk, negedge rst_n) begin
	if(!rst_n)
		PWM2 <= 0;
	else if(rst_PWM2)
		PWM2 <= 0;
	else if(cnt_grt_sum)
	    PWM2 <= 1'b1;
end

always @ (posedge clk, negedge rst_n) begin
	if(!rst_n)
		PWM1 <= 0;
	else if(rst_PWM1)
		PWM1 <= 0;
	else if(cnt_grt_param)
		PWM1 <= 1'b1;

end

///////////
//counter//
///////////
always @ (posedge clk, negedge rst_n)begin
    if(!rst_n)
        cnt <= 12'h000;
    else
        cnt <= cnt + 1;
    end



endmodule