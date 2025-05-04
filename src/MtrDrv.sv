module MtrDrv(
	input clk, rst_n,	
	input signed [11:0]lft_spd,		
	input signed [11:0]rght_spd,	
	input [11:0] vbatt,	
	output lftPWM1, lftPWM2,	
	output rghtPWM1, rghtPWM2	
);
logic signed [11:0] lft_pipe, rght_pipe;
logic signed [12:0] scale_factor, lft_scaled, rght_scaled;
logic signed [23:0]lft_prod, rght_prod, lft_mult, rght_mult, lft_mult_pipe, rght_mult_pipe;
logic [11:0] lft_duty,rght_duty;

always_ff @(posedge clk) begin
	lft_pipe <= lft_spd;
	rght_pipe <= rght_spd;
end

DutyScaleROM iROM(.clk(clk), .batt_level(vbatt[9:4]), .scale(scale_factor));

assign lft_mult = (lft_pipe * scale_factor);
assign rght_mult = (rght_pipe * scale_factor);


always_ff @(posedge clk) begin
	lft_mult_pipe <= lft_mult;
	rght_mult_pipe <= rght_mult;
end

assign lft_prod = lft_mult_pipe >>> 11; // $signed(2048);
assign rght_prod = rght_mult_pipe >>> 11; // $signed(2048);



assign lft_scaled = ((lft_prod[23]) && !(&lft_prod[22:11])) ? 12'h800 :
				((!lft_prod[23]) && (|lft_prod[22:11])) ? 12'h7FF :
				lft_prod[11:0];

assign rght_scaled = ((rght_prod[23]) && !(&rght_prod[22:11])) ? 12'h800 :
				((!rght_prod[23]) && (|rght_prod[22:11])) ? 12'h7FF :
				rght_prod[11:0];
				

assign lft_duty = lft_scaled + 12'h800;
assign rght_duty = -rght_scaled + 12'h800;


PWM12 PWM_lft(.clk(clk), .rst_n(rst_n), .duty(lft_duty), .PWM1(lftPWM1), .PWM2(lftPWM2));
PWM12 PWM_rght(.clk(clk), .rst_n(rst_n), .duty(rght_duty), .PWM1(rghtPWM1), .PWM2(rghtPWM2));

endmodule
