module vga_demo(CLOCK_50, SW, KEY, HEX3, HEX2, HEX1, HEX0,
                VGA_R, VGA_G, VGA_B,
                VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_CLK, LEDR, PS2_CLK, PS2_DAT);
    
    input CLOCK_50;    
    input [9:0] SW;
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
    wire [7:0] X, X_FSM;
    wire [6:0] Y, Y_FSM;
	inout PS2_CLK, PS2_DAT;
	wire start;
	wire plot;
	
	 PS2_Comm comm(
        .CLOCK_50(CLOCK_50),
        .KEY(KEY[0:0]),

        .PS2_CLK(PS2_CLK),
        .PS2_DAT(PS2_DAT),

        .HEX0(HEX0),
        .HEX1(HEX1),
		  .start(start)
    );
	 


   vga_adapter VGA (
       .resetn(KEY[0]),
       .clock(CLOCK_50),
       .colour(colour),
       .x(X),
       .y(Y),
       .plot(plot),
       .VGA_R(VGA_R),
       .VGA_G(VGA_G),
       .VGA_B(VGA_B),
       .VGA_HS(VGA_HS),
       .VGA_VS(VGA_VS),
       .VGA_BLANK_N(VGA_BLANK_N),
       .VGA_SYNC_N(VGA_SYNC_N),
       .VGA_CLK(VGA_CLK),
	   .VGAstate(VGAstate));
	defparam VGA.RESOLUTION = "160x120";
	defparam VGA.MONOCHROME = "FALSE";
	defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
	defparam VGA.BACKGROUND_IMAGE = "title1.colour.mif";
	defparam VGA.BACKGROUND_IMAGE2 = "title2.colour.mif";
	defparam VGA.BACKGROUND_IMAGE3 = "gameScreen.colour.mif";
		  
	wire [1:0] count;
	wire [1:0] VGAstate;
	wire [3:0] state;
	wire dcount, denable, doPlot, objToPlot;
	fsm U1(CLOCK_50, KEY[0], count[0], state, VGAstate, start, dcount, denable, 0, X_FSM, Y_FSM, doPlot, objToPlot); // ~KEY[3]: doPlot, SW[9]: objToPlot
	counter U2(CLOCK_50, KEY[0], count);
	downCounter U3(CLOCK_50, KEY[0], dcount, denable);
	plotObj U4(Y_FSM, X_FSM, doPlot, KEY[0], objToPlot, CLOCK_50, X, Y, plot, colour);
	assign LEDR[9:6] = state[3:0];
		  
endmodule

module fsm(clock, reset, count, state, VGAstate, start, dcount, denable, endGame, x, y, plot, objToPlot);
	input clock, reset, count, start, dcount;
	//T1 - Title Screen W/O Text State, T2 - Title Screen With Text Stete, gameStart - Game Start State
	parameter T1 = 4'b0001, T2 = 4'b0010, gameStart = 4'b0011, DrawB = 4'b0100, DrawBWait = 4'b0101, DrawP = 4'b0110, DrawPWait = 4'b0111;
	parameter DrawH = 4'b1000, DrawHWait = 4'b1001, DrawC = 4'b1010, DrawCWait = 4'b1011, UpdateState = 4'b1100, gameEnd = 4'b1101;
	reg [3:0] currentState, nextState;
	output reg [3:0] state;
	output reg [2:0] VGAstate;
	output reg denable;
	output reg [7:0] x;
	output reg [6:0] y;
	output reg plot, objToPlot;
	input endGame;
	
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

			gameStart: nextState <= DrawB;

			DrawB: if(dcount == 1)
						nextState <= DrawBWait;
					else
						nextState <= DrawB;

			DrawBWait: if(dcount == 0)
							nextState <= DrawP;
						else
							nextState <= DrawBWait;

			DrawP: if(dcount == 1)
						nextState <= DrawPWait;
					else
						nextState <= DrawP;

			DrawPWait: if(dcount == 0)
							nextState <= DrawH;
						else
							nextState <= DrawPWait;
			
			DrawH: if(dcount == 1)
						nextState <= DrawHWait;
					else
						nextState <= DrawH;

			DrawHWait: if(dcount == 0)
							nextState <= DrawC;
						else
							nextState <= DrawHWait;

			DrawC: if(dcount == 1)
						nextState <= DrawCWait;
					else
						nextState <= DrawC;

			DrawCWait: if(dcount == 0)
							nextState <= UpdateState;
						else
							nextState <= DrawCWait;
			
			UpdateState: if(endGame == 1)
							nextState <= gameEnd;
						else
							nextState <= DrawB;
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
	reg [6:0] ballY;
	reg [7:0] hoopX;
	always@(posedge clock)
	begin 
		case (currentState)
			T1: begin
				VGAstate <= 2'b01; //Sets VGA State to Title Screen W/O Text
				denable <= 1'b0;
				hoopX <= 0;
				ballY <= 0; 
			end
		
			T2: VGAstate <= 2'b00; //Sets VGA State to Title Screen With Text

			gameStart: begin
				VGAstate <= 2'b10; //Sets VGA State to Game Screen
				denable <= 1'b0;
			end

			DrawB: denable <= 1'b1;
			DrawBWait:  denable <= 1'b0;
			
			DrawP: begin
					 denable <= 1'b1;
					 plot <= 0;
					 objToPlot <= 1;
					 x <= 8'd30;
					 y <= 7'd100; // update module
					 end
			DrawPWait: begin 
						  denable <= 1'b0;
					     plot <= 1; // starts drawing
					     end

			DrawH: begin
					 denable <= 1'b1;
					 plot <= 0;
					 objToPlot <= 0; // 0 for hoop, 1 for ball
					 x <= hoopX; // update module
					 y <= 7'd60;
					 end
			DrawHWait: begin 
						  denable <= 1'b0;
					     plot <= 1;
					     end

			DrawC: denable <= 1'b1;
			DrawCWait: denable <= 1'b0;
			
			UpdateState: hoopX <= hoopX - 1;
					 
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
			fastcount <= 26'd25000000;
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

module downCounter(clock, reset, dcount, enable);
	input [0:0] reset;
	input clock;
	reg [25:0] fastcount;
	output reg [1:0] dcount;
	input enable;
	
	always@(posedge clock)
	begin
		if (reset[0] == 0 || enable)
			// Resets downcounter if it gets reset or enabled
			// Counts from 500,000 -> 0. Takes around 10ms, faster than the refresh rate of the VGA (60Hz = 16.67ms/refresh)
			fastcount <= 26'd500000;
		else 
			// Every other time the downcounter is counting down
			fastcount <= fastcount - 1'b1;
			
	end
	
	always@(posedge clock)
	begin
		// dcount = 0 if couting down is finished, 1 if down counter is still counting
		if (reset[0] == 0 || enable)
			// Resets dcount to 1 to start count down
			dcount <= 1'b1;
		else if (fastcount == 1'b0)
			// Once the fastcounter is done counting, set dcount to 0
			dcount <= 1'b0;
	end
endmodule

module plotObj(y, x, plot, reset, ObjToPlot, CLOCK_50, VGA_X, VGA_Y, doPlot, VGA_COLOR);
	input CLOCK_50, plot, reset;
	output reg doPlot;
	output reg [7:0] VGA_X;       
	output reg [6:0] VGA_Y;
	input [7:0] x;
	input [6:0] y;
	output reg [2:0] VGA_COLOR;
	input ObjToPlot;

	wire [7:0] X;           // starting x location of object
	wire [6:0] Y;           // starting y location of object
   wire [4:0] XC, YC;      // used to access object memory
   wire Ex, Ey, playerEy;
	
	wire [2:0] VGA_Player;
	wire [2:0] VGA_Hoop;
	
	
	 wire [3:0] playerXC, playerYC; 
	
    // store (x,y) starting location
    regn U1 (y, reset, 1'b1, CLOCK_50, Y);
        defparam U1.n = 7;
		  
    regn U2 (x, reset, 1'b1, CLOCK_50, X);
        defparam U2.n = 8;

    count U3 (CLOCK_50, reset, Ex, XC);    // Hoop column counter
        defparam U3.n = 5;
    
	 
	 count C3 (CLOCK_50, reset, Ex, playerXC);    // player column counter
        defparam C3.n = 4;
		  
	 // enable XC when VGA plotting starts  
    regn U5 (1'b1, reset, plot, CLOCK_50, Ex);
        defparam U5.n = 1;
		  
    count U4 (CLOCK_50, reset, Ey, YC);    // Hoop row counter
        defparam U4.n = 5;
		  
	 count C4 (CLOCK_50, reset, playerEy, playerYC);    // player row counter
        defparam C4.n = 4;
		  
    // enable Y Counters at the end of each object row
    assign Ey = (XC == 5'b11111);
	 assign playerEy = (playerXC == 4'b1111);

    // read a pixel color from object memory
    object_mem U6 ({YC,XC}, CLOCK_50, VGA_Hoop);
	 player_mem U10 ({playerYC,playerXC}, CLOCK_50, VGA_Player);
	 
	 wire [7:0] P_VGA_X, O_VGA_X;
	 wire [6:0] P_VGA_Y, O_VGA_Y;
	 
	 regn U7 (X + XC, reset, 1'b1, CLOCK_50, O_VGA_X);
        defparam U7.n = 8;
    regn U8 (Y + YC, reset, 1'b1, CLOCK_50, O_VGA_Y);
        defparam U8.n = 7;
	 
	 regn P7 (X + playerXC, reset, 1'b1, CLOCK_50, P_VGA_X);
        defparam P7.n = 8;
    regn P8 (Y + playerYC, reset, 1'b1, CLOCK_50, P_VGA_Y);
        defparam P8.n = 7;
	 
	 always@(negedge CLOCK_50) begin
		if(ObjToPlot == 1'b1) begin
			VGA_COLOR <= VGA_Player;
			VGA_X <= P_VGA_X;
			VGA_Y <= P_VGA_Y;
		end
		else begin
			VGA_COLOR <= VGA_Hoop;
			VGA_X <= O_VGA_X;
			VGA_Y <= O_VGA_Y;
		end 
			
	 end
	 
	 // store correct value of doPlot
	 always@(VGA_COLOR) begin
		if(VGA_COLOR == 3'b011)
			doPlot <= 1'b0;
		else
			doPlot <= plot; // only plot if the colour is not blue
	 end
	 
endmodule

module regn(R, Resetn, E, Clock, Q);
    parameter n = 8;
    input [n-1:0] R;
    input Resetn, E, Clock;
    output reg [n-1:0] Q;

    always @(posedge Clock)
        if (!Resetn)
            Q <= 0;
        else if (E)
            Q <= R;
endmodule

// counter
module count (Clock, Resetn, E, Q);
    parameter n = 8;
    input Clock, Resetn, E;
    output reg [n-1:0] Q;

    always @ (posedge Clock)
        if (Resetn == 0)
            Q <= 0;
        else if (E)
            Q <= Q + 1;
endmodule







