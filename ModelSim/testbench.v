`timescale 1ns / 1ps

// module downCounter(clock, reset, dcount, enable);
module testbench ( );

	parameter CLOCK_PERIOD = 10;
	parameter counterP = 100;

   	reg CLOCK_50;
	reg start;
	reg count;
	wire enable;
	wire dcount;
	wire [3:0] state;
	wire [2:0] VGAstate;
	reg [7:0] SW;
	reg [0:0] KEY;
  	wire [6:0] HEX3, HEX2, HEX1, HEX0;
	wire [7:0] VGA_R;
	wire [7:0] VGA_G;
	wire [7:0] VGA_B;
	wire VGA_HS;
	wire VGA_VS;
	wire VGA_BLANK_N;
	wire VGA_SYNC_N;
	wire VGA_CLK;
	reg endGame;


	initial begin
        CLOCK_50 <= 1'b0;
		count <= 1'b0;
		start <= 1'b0;
		endGame <= 1'b0;
	end // initial
	always @ (*)
	begin : Clock_Generator
		#((CLOCK_PERIOD) / 2) CLOCK_50 <= ~CLOCK_50;
	end
	
	always @ (*)
	begin : counterP_Generator
		#((counterP) / 2) count <= ~count;
	end
	
	initial begin
        KEY[0] <= 1'b1; // reset
		#10 KEY[0] <= 1'b0;
		#20 KEY[0] <= 1'b1;
	end // initial

	initial begin
		#200 start <= 1'b1;
		#40 start <= 1'b0;
		#900 endGame <= 1'b1;
	end // initial

	// module fsm(clock, reset, count, state, VGAstate, start, dcount, denable, endGame);	
	downCounter U2(CLOCK_50, KEY[0], dcount, enable);
	fsm U3(CLOCK_50, KEY[0], count, state, VGAstate, start, dcount, enable, endGame);

endmodule
