module RunnerPhysics(clk,RST_n,SS_n,SCLK,MISO,MOSI,INT,lftPWM1,lftPWM2,rghtPWM1,rghtPWM2,
                     IR_lft_en,IR_cntr_en,IR_rght_en,A2D_SS_n,A2D_SCLK,A2D_MOSI,A2D_MISO,
					 hall_n,batt);
  //////////////////////////////////////////////////
  // Model of physics of the MazeRunner and also //
  // models a simple maze as well.              //
  ///////////////////////////////////////////////

  input clk;				// 50MHz clock
  input RST_n;				// unsynchronized raw reset input
  input SS_n;				// active low slave select to inertial sensor
  input SCLK;				// Serial clock
  input MOSI;				// serial data in from monarch
  input IR_lft_en;			// enables IR sensors
  input IR_cntr_en;			// enables IR sensors
  input IR_rght_en;			// enables IR sensors
  input lftPWM1,lftPWM2;	// drive magnitude left motor
  input rghtPWM1,rghtPWM2;	// drive magnitude right motor
  input A2D_SS_n;			// active low serf select for A2D model
  input A2D_SCLK;			// Serial clock A2D
  input A2D_MOSI;			// serial data from MazeRunner (select channel)
  input [11:0] batt;		// battery level.  0xD80 would be nominal level
  
  
  output MISO;				// serial data out to inertial sensor
  output A2D_MISO;			// serial data out (IR sensors & battery)
  output INT;				// inertial reading ready
  output reg hall_n;			// magnet found

  //////////////////////////////////////////////////////////
  // Registers needed for modeling physics of MazeRunner //
  ////////////////////////////////////////////////////////
  reg signed [12:0] alpha_lft,alpha_rght;			// angular acceleration of wheels
  reg signed [15:0] omega_lft,omega_rght;			// angular velocities of wheels
  reg signed [16:0] omega_sum;				        // if sum positive we are moving forward
  reg signed [15:0] heading_v;					// function of omega_rght - omega_lft
  reg signed [19:0] heading_robot;				// angular orientation of robot (starts at zero) integration of heading_v
  reg signed [11:0] ordinal_err;				// use to "leak" robot_heading toward an ordinal direction
  reg [6:0] rand_err;
  reg signed [15:0] gyro_err;
  reg [14:0] xx,yy;  						// board coordinates with 4096X multiplier
  reg [7:0] magnet_pos_xx,magnet_pos_yy;	// magnet position with 16X multiplier
  reg [16:0] omega_prod;
  reg [7:0] dist_increment;
  reg [11:0] lftIR,cntrIR,rghtIR;
  reg [3:0]mazeModel[0:3][0:3];		// stores maze layout.  For each location store if
									// there is a wall present N/S/E/W
  
  /////////////////////////////////////////////
  // Declare internal signals between units //
  ///////////////////////////////////////////
  wire [10:0] mtrL1,mtrL2;		// inversePWM outputs telling motor drive magnitude
  wire [10:0] mtrR1,mtrR2;		// inversePWM outputs telling motor drive magnitude  
  wire calc_physics;			// update the physics model everytime inversePWM refreshes
  
  localparam NOM_IR = 12'hB00;

  logic [2:0]xx_12;
  logic [2:0]yy_12;

  assign xx_12 = xx[14:12];
  assign yy_12 = yy[14:12];
  
  /////////////////////////////////////////////////////
  // Instantiate model of SPI based inertial sensor //
  ///////////////////////////////////////////////////
  SPI_iNEMO4 iNEMO(.SS_n(SS_n),.SCLK(SCLK),.MISO(MISO),.MOSI(MOSI),.INT(INT),.YAW(heading_v)); // +gyro_err));
  
  //////////////////////////////////////////////////////////////
  // Instantiate inverse PWM's to get motor drive magnitudes //
  ////////////////////////////////////////////////////////////
  inverse_PWM12e iMTRL1(.clk(clk),.rst_n(RST_n),.PWM_sig(lftPWM1),.duty_out(mtrL1),.vld(calc_physics));
  inverse_PWM12e iMTRL2(.clk(clk),.rst_n(RST_n),.PWM_sig(lftPWM2),.duty_out(mtrL2),.vld()); 
  inverse_PWM12e iMTRR1(.clk(clk),.rst_n(RST_n),.PWM_sig(rghtPWM1),.duty_out(mtrR1),.vld());
  inverse_PWM12e iMTRR2(.clk(clk),.rst_n(RST_n),.PWM_sig(rghtPWM2),.duty_out(mtrR2),.vld()); 

  /////////////////////////////////////////////
  // Next is modeling physics of MazeRunner //
  ///////////////////////////////////////////
  always @(posedge calc_physics) begin
	alpha_rght = alpha(mtrR2,mtrR1,omega_rght);	// angular accel direct to (duty - k*omega)
    alpha_lft = alpha(mtrL1,mtrL2,omega_lft);		// angular accel direct to (duty - k*omega)
	omega_lft = omega(omega_lft,alpha_lft);		// angular velocity is integral of alpha
	omega_rght = omega(omega_rght,alpha_rght);	// angular velocity is integral of alpha
	omega_sum = omega_lft + omega_rght;             // if just pivoting this is near zero, positive when moving forward
	omega_prod = omega_sum[16:10]*10'd698;				// scaling distance in Physics...found through
	dist_increment = omega_prod[16:9];				// trial and error.
    heading_v = omega_plat(omega_rght,omega_lft);	// angular velocity of platform is function of omegaR - omegaL
	heading_robot = theta_plat(heading_robot,heading_v);	// theta of platform is integration of omega_plat
    rand_err = $random() % 128;				// 7-bit random error
    gyro_err = {{9{rand_err[6]}},rand_err};
	
	/// is MazeRunner close to magnet? ///
	hall_n = ((xx[14:8]>magnet_pos_xx-7'h03) && (xx[14:8]<magnet_pos_xx+7'h03) &&
	          (yy[14:8]>magnet_pos_yy-7'h03) && (yy[14:8]<magnet_pos_yy+7'h03)) ? 1'b0 : 1'b1;
			  
	//// Now update position on board xx,yy based on heading & speed /////
	if ((omega_lft>$signed(16'd1000)) && (omega_rght>$signed(16'd1000))) begin // both wheels moving forward
	  case (heading_robot[19:8]) inside
	    [12'h330:12'h4D0] : begin		//  West
	            ordinal_err = 12'h3FF - heading_robot[19:8];
		    xx = xx - dist_increment;
			if (omega_sum>17'd10000)
		      if (heading_robot[19:8]<12'h3FF) 		// north of pure west
			    yy = yy + (8'h3F - heading_robot[19:12]);
			  else									// south of pure west
			    yy = yy - (heading_robot[19:12] - 8'h3F);
		end
		[12'hB30:12'hCD0] : begin		//  East
	            ordinal_err = 12'hC00 - heading_robot[19:8];
		    xx = xx + dist_increment;
			if (omega_sum>17'd10000)
		      if (heading_robot[19:8]<12'hBFF) 		// south of pure east
			    yy = yy - (8'hBF - heading_robot[19:12]);
			  else									// north of pure east
			    yy = yy + (heading_robot[19:12] - 8'hBF);
		end
		[12'h730:12'h7FF] : begin		// west of pure south
	            ordinal_err = 12'h7FF - heading_robot[19:8];
		    yy = yy - dist_increment;
			if (omega_sum>17'd10000)
			  xx = xx - (8'h7F - heading_robot[19:12]);
        end
		[12'h800:12'h8D0] : begin		// east of pure south
	            ordinal_err = 12'h800 - heading_robot[19:8];
		    yy = yy - dist_increment;
			if (omega_sum>17'd10000)
			  xx = xx + (heading_robot[19:12] - 8'h80);
        end		  
		[12'h000:12'h0D0] : begin						// west of pure north
	            ordinal_err = -heading_robot[19:8];
		    yy = yy + dist_increment;
			if (omega_sum>17'd10000)
			  xx = xx - heading_robot[19:12];
		end
		[12'hF30:12'hFFF] : begin						// east of pure north
	            ordinal_err = -heading_robot[19:8];
		    yy = yy + dist_increment;
			if (omega_sum>17'd10000)
			  xx = xx - {{9{heading_robot[19]}},heading_robot[19:12]};
		end
		default : begin
		  $display("PHYS ERR: not traveling orthogonal direction");
		  ordinal_err = 12'h000;
		end
	  endcase
	  heading_robot = heading_robot + {{2{ordinal_err[11]}},ordinal_err,6'h00};
	end
	
    computeIRs();
		  
  end
	

  initial begin
    alpha_lft = 13'h0000;
	alpha_rght = 13'h0000;
	omega_lft = 16'h0000;
	omega_rght = 16'h0000;
	heading_robot = 20'h00000;		// start North
	xx = 15'h2800;			// start 2.5 squares from left
	yy = 15'h800;			// start 0.5 squares up.
	magnet_pos_xx = 7'h38;	// magnet pos is middle of (3,3)
	magnet_pos_yy = 7'h38;
	mazeModel[0][0] = 4'h5;	// SW
	mazeModel[1][0] = 4'h6;	// SE
	mazeModel[2][0] = 4'h3;	// EW
	mazeModel[3][0] = 4'h7;	// SEW
	mazeModel[0][1] = 4'h3;	// EW
	mazeModel[1][1] = 4'h1;	// W
	mazeModel[2][1] = 4'hA;	// NE
	mazeModel[3][1] = 4'h3;	// EW
	mazeModel[0][2] = 4'h3;	// EW
	mazeModel[1][2] = 4'h9;	// NW
	mazeModel[2][2] = 4'hC;	// NS
	mazeModel[3][2] = 4'h2;	// E
	mazeModel[0][3] = 4'h9; // NW
	mazeModel[1][3] = 4'hC;	// NS
	mazeModel[2][3] = 4'hC;	// NS
	mazeModel[3][3] = 4'hA;	// NE
	cntrIR = 12'hFFF;		// clear to start
	computeIRs();
  end
  
  //////////////////////////////////////////////////////
  // functions used in "physics" computations follow //
  ////////////////////////////////////////////////////
  
  //// Angular acceleration of wheel as function of duty, and omega ////
  function signed [12:0] alpha (input [10:0] duty1, duty2, input signed [15:0] omega1);
    reg [11:0] mag;
	reg [11:0] mag_shaped;
	reg signed [12:0] torque;
	reg [13:0] alpha14bit;

    mag = (duty1>duty2) ? duty1 - duty2 : duty2 - duty1;
	mag_shaped = $sqrt(real'({mag,12'h000})) + {2'b00,mag[11:2]};
    torque = (duty1>duty2) ? mag_shaped : -{1'b0,mag_shaped};
	if (mag_shaped>12'd010)
	  alpha14bit = {torque[12],torque} - {{2{omega1[15]}},omega1[15:4]} - {{4{omega1[15]}},omega1[15:6]};
	else
	  alpha14bit = {torque[12],torque}; 

    alpha = (alpha14bit[13]&~alpha14bit[12]) ? 13'h1000 :
	        (~alpha14bit[13]&alpha14bit[12]) ? 13'h0FFF :
		    alpha14bit[12:0];

  endfunction
 
   //// Angular velocity of wheel as integration of alpha ////
  function signed [15:0] omega (input signed [15:0] omega1, input signed [12:0] torque);
    //// if torque is greater than friction wheel speed changes ////
	reg signed [15:0] intermediate;
	reg [11:0] abs_torque;
	reg [14:0] abs_omega1;
	reg [15:0] friction;
	reg [15:0] friction_min;
	
	abs_torque = (torque[12]) ? -torque : torque;
	abs_omega1 = (omega1[15]) ? -omega1 : omega1;
	// friction = (omega1[15]) ? 16'hFFF8 : 16'h0008;
	friction = (omega1[15]) ? {{6{omega1[15]}},omega1[15:6]} : {6'h00,omega1[15:6]};
        friction_min = (friction[15] && (friction>16'hFFF7)) ? 16'hFFF7 :
		       (!friction[15] && (friction<16'h0009)) ? 16'h0009 :
		       friction;
	
	if ((abs_torque<12'h0040) && (abs_omega1<15'd400))	// at very low torque wheel stops quick
	  omega = omega1 - {{1{omega1[15]}},omega1[15:1]};
	else if (abs_torque>abs_omega1[14:3]) begin
	  intermediate = omega1 + {{5{torque[12]}},torque[12:2]} - friction_min;	// wheel speed integrates
	  if (intermediate>$signed(16'd32700))
	    omega = 16'd32700;
	  else if (intermediate<$signed(-16'd32700))
	    omega = -16'd32700;
	  else
	    omega = intermediate;
	end else if (abs_torque>{1'b0,abs_omega1[14:4]}) begin
	  intermediate = omega1 + {{5{torque[12]}},torque[12:2]} -  // wheel speed integrates
	                          {{6{omega1[15]}},omega1[15:6]} - 	// but back emf in play
							  friction_min;							// and so is friction
	  if (intermediate>$signed(16'd32700))
	    omega = 16'd32700;
	  else if (intermediate<$signed(-16'd32700))
	    omega = -16'd32700;
	  else
	    omega = intermediate;	  
	end else
	  omega = omega1 - {{2{omega1[15]}},omega1[15:2]} - friction_min;	// friction/back emf takes its toll 
  endfunction

   //// Angular position of wheel as integration of omega ////  
  function signed [21:0] theta (input signed [21:0] theta1, input signed [15:0] omega);
	theta = theta1 + {{11{omega[15]}},omega[15:5]};
  endfunction
  
  //// Angular velocity of platform is proportional to omegaR - omegaL ////
  function signed [15:0] omega_plat (input signed [15:0] omegaR,omegaL);
	omega_plat = omegaR - omegaL + {{2{omegaR[15]}},omegaR[15:2]} - {{2{omegaL[15]}},omegaL[15:2]} ; 
  endfunction
  
  
  //// Angle of platform is integration of omega_plat ////
  function signed [19:0] theta_plat (input signed [19:0] theta_plat1,input signed [15:0] omega_plat1);
        reg [31:0] prod;
	prod = omega_plat1*$signed(16'h1FA0);
	theta_plat = theta_plat1 + {{4{prod[31]}},prod[31:16]};
  endfunction
  
  task computeIRs();
    reg [3:0] y_indx,x_indx;
	
	case (heading_robot[19:8]) inside
	    [12'h321:12'h4DF] : begin		//  West
		  x_indx = (xx[13:8] + 6'h5)>>4;	// slow onset of no fence detect
		  if (mazeModel[xx[13:12]][yy[13:12]]&4'h1) begin // forward
		    if (xx[11:0]<12'h800)
			  cntrIR = 12'hC00;			// obstruction ahead
		  end else
		    cntrIR = 12'hFFF;
		  if (mazeModel[x_indx][yy[13:12]]&4'h8)  	// right
		    rghtIR = NOM_IR - (yy[11:0] - 12'h800);
		  else
		    rghtIR = 12'hFFF;
		  if (mazeModel[x_indx][yy[13:12]]&4'h4)  	// left
		    lftIR = NOM_IR + (yy[11:0] - 12'h800);
		  else
		    lftIR = 12'hFFF;
		end
		[12'hB21:12'hCDF] : begin		//  East
		  x_indx = (xx[13:8] - 6'h5)>>4;	// slow onset of no fence detect
		  if (mazeModel[xx[13:12]][yy[13:12]]&4'h2) begin // forward
		    if (xx[11:0]>12'h800)
			  cntrIR = 12'hC00;			// obstruction ahead
		  end else
		    cntrIR = 12'hFFF;
		  if (mazeModel[x_indx][yy[13:12]]&4'h4)  	// right
		    rghtIR = NOM_IR + (yy[11:0] - 12'h800);
		  else
		    rghtIR = 12'hFFF;
		  if (mazeModel[x_indx][yy[13:12]]&4'h8)  	// left
		    lftIR = NOM_IR - (yy[11:0] - 12'h800);
		  else
		    lftIR = 12'hFFF;
		end
		[12'h721:12'h8DF] : begin		//  south
		  y_indx = (yy[13:8] + 6'h5)>>4;	// slow onset of no fence detect
		  if (mazeModel[xx[13:12]][yy[13:12]]&4'h4) begin // forward
		    if (yy[11:0]<12'h800)
			  cntrIR = 12'hC00;			// obstruction ahead
		  end else
		    cntrIR = 12'hFFF;
		  if (mazeModel[xx[13:12]][y_indx]&4'h1)  	// right
		    rghtIR = NOM_IR + (xx[11:0] - 12'h800);
		  else
		    rghtIR = 12'hFFF;
		  if (mazeModel[xx[13:12]][y_indx]&4'h2)  	// left
		    lftIR = NOM_IR - (xx[11:0] - 12'h800);
		  else
		    lftIR = 12'hFFF;
        end
	        [12'h000:12'h0DF] : begin 		// West of pure North
		  y_indx = (yy[13:8] - 6'h5)>>4;	// slow onset of no fence detect
		  if (mazeModel[xx[13:12]][yy[13:12]]&4'h8) begin // forward
		    if (yy[11:0]>12'h800)
			  cntrIR = 12'hC00;			// obstruction ahead
		  end else
		    cntrIR = 12'hFFF;
		  if (mazeModel[xx[13:12]][y_indx]&4'h2)  	// right
		    rghtIR = NOM_IR - (xx[11:0] - 12'h800);
		  else
		    rghtIR = 12'hFFF;
		  if (mazeModel[xx[13:12]][y_indx]&4'h1)		 // left
		    lftIR = NOM_IR + (xx[11:0] - 12'h800);
		  else
		    lftIR = 12'hFFF;		  
		end
                [12'hF21:12'hFFF] : begin 		// East of pure North
		  y_indx = (yy[13:8] - 6'h5)>>4;	// slow onset of no fence detect
		  if (mazeModel[xx[13:12]][yy[13:12]]&4'h8) begin // forward
		    if (yy[11:0]>12'h800)
			  cntrIR = 12'hC00;			// obstruction ahead
		  end else
		    cntrIR = 12'hFFF;
		  if (mazeModel[xx[13:12]][y_indx]&4'h2)  	// right
		    rghtIR = NOM_IR - (xx[11:0] - 12'h800);
		  else
		    rghtIR = 12'hFFF;
		  if (mazeModel[xx[13:12]][y_indx]&4'h1)		 // left
		    lftIR = NOM_IR + (xx[11:0] - 12'h800);
		  else
		    lftIR = 12'hFFF;		  
		end
		default : cntrIR = 12'hFFF;			// if not ordinal dir then make center open
	endcase
  endtask
  
  //////////////////////////////////////////////////////////
  // Instantiate Model of A2D for IR sensors and battery //
  ////////////////////////////////////////////////////////
  ADC128S_FC iA2D(.clk(clk),.rst_n(RST_n),.SS_n(A2D_SS_n),.SCLK(A2D_SCLK),
             .MISO(A2D_MISO),.MOSI(A2D_MOSI),.IR_lft(lftIR),.IR_cntr(cntrIR),
			 .IR_rght(rghtIR),.batt(batt));	
			 
endmodule

///////////////////////////////////////////////////
// Inverse PWM defined below for easy reference //
/////////////////////////////////////////////////
module inverse_PWM12e(clk,rst_n,PWM_sig,duty_out,vld);

  input clk,rst_n;
  input PWM_sig;
  output reg [10:0] duty_out;		// dropping lowest bit so only 11-bit output
  output reg vld;
  
  reg [11:0] pwm_cnt;
  reg [11:0] per_cnt;
  
  //////////////////////////////////////////
  // Count the duty cycle of the PWM_sig //
  ////////////////////////////////////////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
	  pwm_cnt <= 12'h000;
	else if (&per_cnt)
	  pwm_cnt <= 12'h000;
	else if (PWM_sig)
	  pwm_cnt <= pwm_cnt + 1;
	  
  ///////////////////////////////////////
  // Need to count the PWM period off //
  /////////////////////////////////////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
	  per_cnt <= 12'h000;
	else
	  per_cnt <= per_cnt + 1;

  ////////////////////////////////////////////////////
  // Buffer pwm_cnt in output register so it holds //
  //////////////////////////////////////////////////  
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
	  duty_out <= 11'h000;
	else if (&per_cnt)
	  duty_out <= pwm_cnt[11:1];
	  
  ///////////////////////////////////////
  // Pulse vld when new reading ready //
  /////////////////////////////////////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
	  vld <= 1'b0;
	else
	  vld <= &per_cnt;
	  
endmodule

  
