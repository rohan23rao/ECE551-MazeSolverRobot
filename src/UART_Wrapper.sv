module UART_Wrapper(clk,rst_n,RX,TX,cmd_rdy,clr_cmd_rdy,trmt,resp,tx_done,cmd);

input clk,rst_n;
input RX,trmt,clr_cmd_rdy;
input [7:0] resp;
output TX,tx_done;
output reg cmd_rdy;
output reg [15:0] cmd;

logic rx_rdy,clr_rx_rdy,en_hld,set_rdy;
logic [7:0]rx_data,high_byte;
	
typedef enum logic {IDLE, LOW} state_t;
state_t state, nxt_state;
	
// UART Instantiation
UART iUART(.clk(clk), .rst_n(rst_n), .RX(RX), .TX(TX), .rx_rdy(rx_rdy), .clr_rx_rdy(clr_rx_rdy), .rx_data(rx_data), .trmt(trmt), .tx_data(resp), .tx_done(tx_done));
	
// state flop
always_ff @(posedge clk, negedge rst_n)
	if(!rst_n)
		state <= IDLE;
	else
		state <= nxt_state;
	
always_comb begin
	// default SM output
	en_hld = 1'b0;
	clr_rx_rdy = 1'b0;
	set_rdy = 1'b0;
	nxt_state = state;

		case(state)
			IDLE: begin
				if(rx_rdy) begin
					en_hld = 1'b1;
					clr_rx_rdy = 1'b1;
					nxt_state = LOW;
				end
			end
			LOW:
				if(rx_rdy) begin
					set_rdy = 1'b1;
					clr_rx_rdy = 1'b1;
					nxt_state = IDLE;
				end
			
		endcase
	end
	
// select higher byte
always_ff @(posedge clk)
	if(en_hld)
		high_byte <= rx_data;
	
// vector concatenation of both data to form cmd
assign cmd = {high_byte, rx_data};
	
// SR flop for setting/resetting cmd_rdy signal
always_ff @(posedge clk, negedge rst_n)
	if(!rst_n)
		cmd_rdy <= 1'b0;
	else if(clr_cmd_rdy)
		cmd_rdy <= 1'b0;
	else if(set_rdy)
		cmd_rdy <= 1'b1;
	
endmodule
