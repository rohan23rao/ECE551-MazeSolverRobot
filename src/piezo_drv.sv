module piezo_drv
	#(parameter FAST_SIM = 1) (
	input clk, rst_n,
	input fanfare, batt_low,
	output logic piezo, piezo_n
);
	// note frequency
    localparam G6 = 15'd31888;        // 50 MHz / 1568
    localparam C7 = 15'd23890;        // 50 MHz / 2093
    localparam E7 = 15'd18960;        // 50 MHz / 2637
    localparam G7 = 15'd15944;        // 50 MHz / 3136

    // half of frequency to maintain a duty of 50
    localparam G6_MID = 15'd15944;
    localparam C7_MID = 15'd11945;
    localparam E7_MID = 15'd9480;
    localparam G7_MID = 15'd7972;

    // state
    typedef enum logic [2:0] { IDLE,NOTE1,NOTE2,NOTE3,NOTE4,NOTE5,NOTE6} state_t;
    state_t state, nxt_state;

    // SM outputs
    logic hold,set_piezo,rst_piezo,clr_d_cnt,clr_f_cnt;


    // frequency
    typedef enum logic [1:0] { freq_G6,freq_C7,freq_E7,freq_G7 } freq_t;
    freq_t sel_freq;

    // duration
    typedef enum logic [1:0] { DUR_1,DUR_2,DUR_3,DUR_4 } dur_t;
    dur_t sel_dur;

    // internal signals
    logic [24:0] dur_full;
    logic [14:0] freq_full, freq_threshold;

    // counters for frequency and duration
    logic [24:0] dur_cnt;
    logic [14:0] freq_cnt;


    assign freq_full =      (sel_freq == freq_G6) ? G6 :
                            (sel_freq == freq_C7) ? C7 :
                            (sel_freq == freq_E7) ? E7 :
                            (sel_freq == freq_G7) ? G7 : 15'h7FFF;
    
    assign freq_threshold = (sel_freq == freq_G6) ? G6_MID :
                            (sel_freq == freq_C7) ? C7_MID :
                            (sel_freq == freq_E7) ? E7_MID :
                            (sel_freq == freq_G7) ? G7_MID : 15'h3FFF;

    

    assign dur_full =       (sel_dur == DUR_1) ? 25'h7FFFFF : 
                            (sel_dur == DUR_2) ? 25'hBFFFFF : 
                            (sel_dur == DUR_3) ? 25'h3FFFFF : 
                            (sel_dur == DUR_4) ? 25'hFFFFFF : 25'hFFFFFF;


    // duration counters
    generate if(FAST_SIM)
        always_ff @(posedge clk, negedge rst_n)
            if(!rst_n)
                dur_cnt <= 0;
            else if(clr_d_cnt)
                dur_cnt <= 0;
            else
                dur_cnt <= dur_cnt + 16;
    else
        always_ff @(posedge clk, negedge rst_n)
            if(!rst_n)
                dur_cnt <= 0;
            else if(clr_d_cnt)
                dur_cnt <= 0;
            else
                dur_cnt <= dur_cnt + 1;
    endgenerate

    // frequency counter
    always_ff @(posedge clk, negedge rst_n)
        if(!rst_n)
            freq_cnt <= 0;
        else if(clr_f_cnt || (freq_cnt == freq_full))
            freq_cnt <= 0;
        else
            freq_cnt <= freq_cnt + 1;


    // state register
    always_ff @(posedge clk, negedge rst_n)
        if(!rst_n)
            state <= IDLE;
        else
            state <= nxt_state;
   

   
    // state transitions
    always_comb begin
        nxt_state = state;
        clr_d_cnt = 0;
        clr_f_cnt = 0;
        hold = 0;
        set_piezo = 0;
        rst_piezo = 0;
        sel_freq = freq_G6;
        sel_dur = DUR_1;

        case(state)
            // IDLE
            default: begin
                hold = 1;
                clr_d_cnt = 1;
                clr_f_cnt = 1;
                rst_piezo = 1;
				if(batt_low | fanfare)
					nxt_state = NOTE1;
            end

            // G6 for 2^23 clocks
            NOTE1: begin
                sel_freq = freq_G6;
                sel_dur = DUR_1;
                if(freq_cnt < freq_threshold) begin
                    set_piezo = 1;
                end 
                else begin
                    rst_piezo = 1;
                end
                if(dur_cnt >= dur_full) begin
                    clr_d_cnt = 1;
                    clr_f_cnt = 1;
                    nxt_state = NOTE2;
                end
            end

            // C7 for 2^23 clocks
            NOTE2: begin
                sel_freq = freq_C7;
                sel_dur = DUR_1;
                if(freq_cnt < freq_threshold) begin
                    set_piezo = 1;
                end 
                else begin
                    rst_piezo = 1;
                end
                if(dur_cnt >= dur_full) begin
                    clr_d_cnt = 1;
                    clr_f_cnt = 1;
					nxt_state = NOTE3;
                end
            end

            // E7 for 2^23 clocks
            NOTE3: begin
                sel_freq = freq_E7;
                sel_dur = DUR_1;
                if(freq_cnt < freq_threshold) begin
                    set_piezo = 1;
                end 
                else begin
                    rst_piezo = 1;
                end
                if(dur_cnt >= dur_full) begin
                    clr_d_cnt = 1;
                    clr_f_cnt = 1;
					if(batt_low) begin
						nxt_state = NOTE1;
					end
					else begin
						nxt_state = NOTE4;
					end
						
                end
            end

            // G7 for 2^23+2^22 clocks
            NOTE4: begin
                sel_freq = freq_G7;
                sel_dur = DUR_2;
                if(freq_cnt < freq_threshold) begin
                    set_piezo = 1;
                end 
                else begin
                    rst_piezo = 1;
                end
                if(dur_cnt >= dur_full) begin
                    clr_d_cnt = 1;
                    clr_f_cnt = 1;
					nxt_state = NOTE5;
                end
            end

            // E7 for 2^22 clocks
            NOTE5: begin
                sel_freq = freq_E7;
                sel_dur = DUR_3;
                if(freq_cnt < freq_threshold) begin
                    set_piezo = 1;
                end 
                else begin
                    rst_piezo = 1;
                end
                if(dur_cnt >= dur_full) begin
                    clr_d_cnt = 1;
                    clr_f_cnt = 1;
                    nxt_state = NOTE6;
                end
            end

            // G7 for 2^24 clocks
            NOTE6: begin
                sel_freq = freq_G7;
                sel_dur = DUR_4;
                if(freq_cnt < freq_threshold) begin
                    set_piezo = 1;
                end 
                else begin
                    rst_piezo = 1;
                end
                if(dur_cnt >= dur_full) begin
                    clr_d_cnt = 1;
                    clr_f_cnt = 1;
                    nxt_state = IDLE;
                end
            end

        endcase

    end

    // SR flop
    always_ff @(posedge clk, negedge rst_n)
        if(!rst_n)
            piezo <= 0;
        else if(rst_piezo)
            piezo <= 0;
        else if(set_piezo)
            piezo <= 1;
        
    assign piezo_n = (hold) ? 0 : ~piezo;

endmodule