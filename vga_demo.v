/*
*   Displays a coloured pattern on the VGA output, which is a large green triangle.
*   Then, allows the user to draw coloured pixels at any x, y coordinates. To set
*   a coordinate, first place the desired value of y onto SW[6:0] and then press KEY[1]
*   to store this value into a register. Next, place the desired value of x onto SW[7:0]
*   and then press KEY[2] to store this into a register. Finally, place the desired 
*   3-bit colour onto SW[2:0] and press KEY[3] to draw the selected pixel. The x 
*   coordinate is displayed (in hexadecimal) on HEX3-2, and y coordinate on HEX1-0.
*/
module vga_demo(CLOCK_50, SW, KEY, HEX3, HEX2, HEX1, HEX0,
                VGA_R, VGA_G, VGA_B,
                VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_CLK, LEDR, PS2_CLK, PS2_DAT);
    
    input CLOCK_50;    
    input [7:0] SW;
	 output [9:0] LEDR;
    input [3:0] KEY;
    output [6:0] HEX3, HEX2, HEX1, HEX0;
    output [7:0] VGA_R;
    output [7:0] VGA_G;
    output [7:0] VGA_B;
    output VGA_HS;
    output VGA_VS;
    output VGA_BLANK_N;
    output VGA_SYNC_N;
    output VGA_CLK;    
    wire [2:0] colour;
    wire [7:0] X;
    wire [6:0] Y;
    assign colour = 0;
	 inout PS2_CLK, PS2_DAT;
	 wire start;
	 
	 PS2_Comm comm(
        .CLOCK_50(CLOCK_50),
        .KEY(KEY[0:0]),

        .PS2_CLK(PS2_CLK),
        .PS2_DAT(PS2_DAT),

        .HEX0(HEX0),
        .HEX1(HEX1),
		  .start(start)
    );
	 


//    vga_adapter VGA (
//        .resetn(KEY[0]),
//        .clock(CLOCK_50),
//        .colour(colour),
//        .x(X),
//        .y(Y),
//        .plot(~KEY[3]),
//        .VGA_R(VGA_R),
//        .VGA_G(VGA_G),
//        .VGA_B(VGA_B),
//        .VGA_HS(VGA_HS),
//        .VGA_VS(VGA_VS),
//        .VGA_BLANK_N(VGA_BLANK_N),
//        .VGA_SYNC_N(VGA_SYNC_N),
//        .VGA_CLK(VGA_CLK),
//		  .VGAstate(VGAstate));
//        defparam VGA.RESOLUTION = "160x120";
//        defparam VGA.MONOCHROME = "FALSE";
//        defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
//        defparam VGA.BACKGROUND_IMAGE = "title1.colour.mif";
//		  defparam VGA.BACKGROUND_IMAGE2 = "title2.colour.mif";
//		  defparam VGA.BACKGROUND_IMAGE3 = "gameScreen.colour.mif";
		  
	wire [1:0] count;
	wire [1:0] VGAstate;
	wire [3:0] state;
	fsm U1(CLOCK_50, KEY[0], SW[3], count[0], state, VGAstate, start);
	counter U2(CLOCK_50, KEY[0], count);
	
	assign LEDR[1:0] = count[1:0];
	assign LEDR[9:6] = state[3:0];
		  
endmodule


//module hex7seg (hex, display);
//    input [3:0] hex;
//    output [6:0] display;
//
//    reg [6:0] display;
//
//    /*
//     *       0  
//     *      ---  
//     *     |   |
//     *    5|   |1
//     *     | 6 |
//     *      ---  
//     *     |   |
//     *    4|   |2
//     *     |   |
//     *      ---  
//     *       3  
//     */
//    always @ (hex)
//        case (hex)
//            4'h0: display = 7'b1000000;
//            4'h1: display = 7'b1111001;
//            4'h2: display = 7'b0100100;
//            4'h3: display = 7'b0110000;
//            4'h4: display = 7'b0011001;
//            4'h5: display = 7'b0010010;
//            4'h6: display = 7'b0000010;
//            4'h7: display = 7'b1111000;
//            4'h8: display = 7'b0000000;
//            4'h9: display = 7'b0011000;
//            4'hA: display = 7'b0001000;
//            4'hB: display = 7'b0000011;
//            4'hC: display = 7'b1000110;
//            4'hD: display = 7'b0100001;
//            4'hE: display = 7'b0000110;
//            4'hF: display = 7'b0001110;
//        endcase
//endmodule


module fsm(clock, reset, switch, count, state, VGAstate, start);
	input clock, reset, switch, count, start;
	parameter T1 = 4'b0001, T2 = 4'b0010, gameStart = 4'b0011;
	reg [3:0] currentState, nextState;
	output reg [3:0] state;
	output reg [2:0] VGAstate;
	always@ (posedge clock)
		begin: state_table
		case (currentState)
			T1: if (count == 0)
					nextState <= T2;
				 else if (start == 1)
					nextState <= gameStart;
				
				 else 
					nextState <= T1;
			
			T2: if (count == 1) 
					nextState <= T1;
					
				 else if (start == 1)
					nextState <= gameStart;
					
				else
					nextState <= T2;
		endcase
		end
	always @(posedge clock)
	begin: state_FFs
		state <= currentState;
		if (!reset)
			currentState <= T1;
		else
			currentState <= nextState;
	end
	
	always@(posedge clock)
	begin
		case (currentState)
			T1: VGAstate <= 2'b01;
			T2: VGAstate <= 2'b00;
			gameStart: VGAstate <= 2'b10;
		endcase
	end
	
endmodule


module counter(clock, reset, count);
	input [0:0] reset;
	input clock;
	reg [25:0] fastcount;
	output reg [1:0] count;
	wire enable;
	
	assign enable = (fastcount == 26'd0) ? 1 : 0; 
	always@(posedge clock)
	begin
		if (reset[0] == 0 || enable)
			// reset to initial image with 
			fastcount <= 26'd4;
		else 
			fastcount <= fastcount - 1'b1;
			
	end
	
	always@(posedge clock)
	begin
		if (reset[0] == 0 || count == 2'd2)
		// display image with play text	
			count <= 0;
		else if (enable == 1)
			// display image without play text
			count <= count + 1;
	end
endmodule






