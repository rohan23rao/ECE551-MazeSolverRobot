`default_nettype none
module UART_rx
(
input wire clk, rst_n, RX, clr_rdy,
output logic [7:0] rx_data,
output logic rdy
);



logic shift;
    //outputs of state machine
    logic start;
    logic receiving;
    logic set_rdy;

    logic [8:0] rx_shft_reg; //9 bits old signal
    logic [11:0] baud_cnt;
    logic [3:0] bit_cnt;

    logic RX_FF2; //back to back flops for meta-stability
    logic RX_FF1;

// double flop RX to avoid meta-stability
	always_ff @(posedge clk, negedge rst_n)begin
		if(!rst_n) begin
			// pre set RX_sync for UART
			RX_FF1 <= 1'b1;
			RX_FF2 <= 1'b1;
		end else begin
			RX_FF1 <= RX;
			RX_FF2 <= RX_FF1;
		end
end
// the shift register	
	
always_ff @(posedge clk)begin
		if(shift)
			rx_shft_reg <= {RX_FF2, rx_shft_reg[8:1]};		// append the data with a start bit
end
	assign rx_data = rx_shft_reg[7:0];		// output the received byte
// the baud counter
always_ff @(posedge clk) begin
		if(start)
			baud_cnt <= 12'd1302;		// set the baud counter to half of a baud period at the start of a receiving
		else if(shift)
			baud_cnt <= 12'd2604;		// set the baud counter to the full baud period when shifting
		else if(receiving)
			baud_cnt <= baud_cnt - 1;		// count up when transmitting
			
end			
    assign shift = ~|baud_cnt; //shift when baud_cnt is zero	

// The bit counter
always_ff @(posedge clk) begin
        if(start)
            bit_cnt <= 4'h0;
        else if(shift)
            bit_cnt <= bit_cnt + 1;
    end
	
	
typedef enum reg { IDLE, RX_STATE } state_t;
    state_t state, nxt_state;
    always_ff @( posedge clk, negedge rst_n ) begin
        if(!rst_n)
            state <= IDLE;
        else
            state <= nxt_state;
    end
always_comb begin
        start = 1'b0;
        receiving = 1'b0;
        set_rdy = 1'b0;
        nxt_state = state; 
		
		
	case(state)
		RX_STATE: begin
			receiving = 1'b1;
			if(bit_cnt == 4'd10) begin		// finish receiving when all 10 bits are received
				set_rdy = 1'b1;
				nxt_state = IDLE;
			end
		end
		default:		
			if(RX_FF2 == 1'b0) begin			// wait until RX is low to begin receiving
				start = 1'b1;
				nxt_state = RX_STATE;
			end
	endcase
end

// S/R reg to set TX_done
always_ff @(posedge clk, negedge rst_n)begin
		if(!rst_n)
			rdy <= 1'b0;
		else if(start | clr_rdy)
			rdy <= 1'b0;
		else if(set_rdy)
			rdy <= 1'b1;
end	
endmodule
`default_nettype wire