module DutyScaleROM(clk,batt_level,scale);

  input clk;
  input [5:0] batt_level;
  output [12:0] scale;
  
  reg [11:0]scaleMem[0:63];
  reg [11:0] entry;
  
  assign scale = {1'b0,entry};	// make it 13-bit positive number
  
 always_ff @(posedge clk) begin
   case(batt_level)
     8'h00 : entry <= 12'h91B;
     8'h01 : entry <= 12'h90F;
     8'h02 : entry <= 12'h903;
     8'h03 : entry <= 12'h8F8;
     8'h04 : entry <= 12'h8EC;
     8'h05 : entry <= 12'h8E0;
     8'h06 : entry <= 12'h8D5;
     8'h07 : entry <= 12'h8C9;
     8'h08 : entry <= 12'h8BE;
     8'h09 : entry <= 12'h8B3;
     8'h0A : entry <= 12'h8A8;
     8'h0B : entry <= 12'h89D;
     8'h0C : entry <= 12'h892;
     8'h0D : entry <= 12'h888;
     8'h0E : entry <= 12'h87D;
     8'h0F : entry <= 12'h872;
     8'h10 : entry <= 12'h868;
     8'h11 : entry <= 12'h85E;
     8'h12 : entry <= 12'h854;
     8'h13 : entry <= 12'h849;
     8'h14 : entry <= 12'h83F;
     8'h15 : entry <= 12'h836;
     8'h16 : entry <= 12'h82C;
     8'h17 : entry <= 12'h822;
     8'h18 : entry <= 12'h818;
     8'h19 : entry <= 12'h80F;
     8'h1A : entry <= 12'h805;
     8'h1B : entry <= 12'h7FC;
     8'h1C : entry <= 12'h7F3;
     8'h1D : entry <= 12'h7E9;
     8'h1E : entry <= 12'h7E0;
     8'h1F : entry <= 12'h7D7;
     8'h20 : entry <= 12'h7CE;
     8'h21 : entry <= 12'h7C5;
     8'h22 : entry <= 12'h7BD;
     8'h23 : entry <= 12'h7B4;
     8'h24 : entry <= 12'h7AB;
     8'h25 : entry <= 12'h7A3;
     8'h26 : entry <= 12'h79A;
     8'h27 : entry <= 12'h792;
     8'h28 : entry <= 12'h789;
     8'h29 : entry <= 12'h781;
     8'h2A : entry <= 12'h779;
     8'h2B : entry <= 12'h771;
     8'h2C : entry <= 12'h769;
     8'h2D : entry <= 12'h761;
     8'h2E : entry <= 12'h759;
     8'h2F : entry <= 12'h751;
     8'h30 : entry <= 12'h749;
     8'h31 : entry <= 12'h741;
     8'h32 : entry <= 12'h73A;
     8'h33 : entry <= 12'h732;
     8'h34 : entry <= 12'h72B;
     8'h35 : entry <= 12'h723;
     8'h36 : entry <= 12'h71C;
     8'h37 : entry <= 12'h714;
     8'h38 : entry <= 12'h70D;
     8'h39 : entry <= 12'h706;
     8'h3A : entry <= 12'h6FF;
     8'h3B : entry <= 12'h6F7;
     8'h3C : entry <= 12'h6F0;
     8'h3D : entry <= 12'h6E9;
     8'h3E : entry <= 12'h6E2;
     8'h3F : entry <= 12'h6DB;
   endcase
 end

endmodule
