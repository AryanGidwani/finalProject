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
    
    wire [7:0] X_FSM, objX, BGX;
    wire [6:0] Y_FSM, objY, BGY;
    inout PS2_CLK, PS2_DAT;
    wire start, moveUp, moveDown;
    wire BGPlotVGA, objPlotVGA;
    
    reg [2:0] colour;
    reg [7:0] X;
    reg [6:0] Y;
    reg plot;
    
    wire [2:0] obj_C;
    
     PS2_Comm comm(
        .CLOCK_50(CLOCK_50),
        .KEY(KEY[0:0]),
 
        .PS2_CLK(PS2_CLK),
        .PS2_DAT(PS2_DAT),
          .start(start),
          .moveUp(moveUp),
			 .moveDown(moveDown)
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
	 defparam VGA.BACKGROUND_IMAGE4 = "gameOver.colour.mif";
          
    wire [1:0] count;
    wire [1:0] VGAstate;
    wire [3:0] state;
    wire dcount, denable, objPlot, objToPlot, BGPlot;
    wire resetCountersFSM, resetCounters;
	 wire [25:0] cyc; //cylces
	 wire [1:0] HoopPos;
	 wire countScore;
	 wire [5:0] val, timerVal;
    wire scoreEnable, gameStarted, gameEnded, resetTimer;
    fsm U1(CLOCK_50, KEY[0], count[0], state, VGAstate, start, dcount, denable, gameEnded, X_FSM, Y_FSM, objPlot, BGPlot, objToPlot, cyc, moveUp, moveDown, HoopPos, scoreEnable, gameStarted, resetTimer); // ~KEY[3]: doPlot, SW[9]: objToPlot
    counter U2(CLOCK_50, KEY[0], count);
    downCounter U3(CLOCK_50, KEY[0], dcount, denable, cyc);
    plotObj U4(Y_FSM, X_FSM, objPlot, KEY[0], objToPlot, CLOCK_50, objX, objY, objPlotVGA, obj_C);
    drawBG U5(BGPlot, CLOCK_50, KEY[0], BGX, BGY, BGPlotVGA);
	 
	 randomCount random(CLOCK_50, KEY[0], count, HoopPos);
	 scoreCounter score(CLOCK_50, KEY[0], scoreEnable, val);
	 gameTimer gameTime(CLOCK_50, KEY[0] & resetTimer, gameStarted, timerVal, gameEnded);
	 hex7seg H1(timerVal[3:0], HEX0);
	 hex7seg H2({2'b00, timerVal[5:4]}, HEX1);
	 
	 
	 
	 assign LEDR[5:0] = val;
    assign LEDR[6] = scoreEnable;
	 assign LEDR[9] = gameEnded;
    always@(posedge CLOCK_50) begin
        if(objPlot == 1) begin
            X <= objX;
            Y <= objY;
            plot <= objPlotVGA;
            colour <= obj_C;
        end
        else if(BGPlot == 1) begin
            X <= BGX;
            Y <= BGY;
            plot <= BGPlotVGA;
            colour <= 3'b011;
        end
		  else plot <= 0;
    end
          
endmodule
 
module fsm(clock, reset, count, state, VGAstate, start, dcount, denable, endGame, x, y, plot, BGplot, objToPlot, cycles, moveUp, moveDown, HoopPos, scoreEnable, gameStarted, resetTimer);
    input clock, reset, count, start, dcount, moveUp, moveDown;
    //T1 - Title Screen W/O Text State, T2 - Title Screen With Text Stete, gameStart - Game Start State
    parameter T1 = 4'b0001, T2 = 4'b0010, gameStart = 4'b0011, DrawB = 4'b0100, DrawBWait = 4'b0101, DrawP = 4'b0110, DrawPWait = 4'b0111;
    parameter DrawH = 4'b1000, DrawHWait = 4'b1001, DrawC = 4'b1010, DrawCWait = 4'b1011, UpdateState = 4'b1100, UpdateWState = 4'b1101, gameEnd = 4'b1110;
	 parameter background = 26'd19225, ball = 26'd261, hoop = 26'd1029, update = 26'd813000; // paramaters used for object down counter
    reg [3:0] currentState, nextState;
    output reg [3:0] state;
    output reg [2:0] VGAstate;
    output reg denable;
    output reg [7:0] x;
    output reg [6:0] y;
    output reg plot, objToPlot, BGplot, gameStarted, resetTimer;
	 output reg [25:0] cycles;
    input endGame; 
	 input [1:0] HoopPos;
    
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
            // alternate values of dcount for drawing object and wait states
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
            
            UpdateState: if(dcount == 1)
                            nextState <= UpdateWState;
                        else
                            nextState <= UpdateState;
				UpdateWState: if (dcount == 0) // delay time
									 nextState <= DrawB;
								  else if (endGame == 1'b1)
									 nextState <= gameEnd;
								  else 
									 nextState <= UpdateWState;
				
				
        endcase
        end
    // wait states occur in between drawing objects
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
	 reg [6:0] hoopY;
	 reg check;
	 output reg scoreEnable;
    always@(posedge clock)
    begin 
        case (currentState)
            T1: begin
                VGAstate <= 2'b01; //Sets VGA State to Title Screen W/O Text
                denable <= 1'b0;
                hoopX <= 0;
                ballY <= 7'd90;
                plot <= 0;
                BGplot <= 0; 
					 check <= 0;
					 scoreEnable <= 0;
					 gameStarted <= 0;
					 resetTimer <= 0; // 0 resets it
            end
        
            T2: VGAstate <= 2'b00; //Sets VGA State to Title Screen With Text
 
            gameStart: begin
                VGAstate <= 2'b10; //Sets VGA State to Game Screen
                denable <= 1'b0;
                hoopX <= 0;
                ballY <= 0;
                plot <= 0;
                BGplot <= 0;
					 gameStarted <= 1'b1;
            end
 
            DrawB: begin 
                denable <= 1'b1;
                BGplot <= 1'b0;
                plot <= 0;
					 cycles <= background;
					 resetTimer <= 1;
            end
            
            DrawBWait:  begin 
                denable <= 1'b0;
                BGplot <= 1'b1; // plot background
            end
            
            DrawP: begin
                     denable <= 1'b1;
                     plot <= 0;
                     BGplot <= 0;
                     objToPlot <= 1; // 1 for ball
                     x <= 8'd30;
                     y <= ballY; // update module
							cycles <= ball;
                     end
            DrawPWait: begin 
                          denable <= 1'b0;
                         plot <= 1; // starts drawing
                         end
 
            DrawH: begin
                     denable <= 1'b1;
                     plot <= 0;
                     objToPlot <= 0; // 0 for hoop 
                     x <= hoopX; // update module
                     y <= hoopY; // y [
							cycles <= hoop;
                     end
            DrawHWait: begin 
                          denable <= 1'b0;
                         plot <= 1;
                         end
 
            DrawC: denable <= 1'b1;
            DrawCWait: denable <= 1'b0;
            
            UpdateState: begin
									denable <= 1'b1;
									cycles <= update;
									check <= check + 1;
									if (check == 1'b1)
										hoopX <= hoopX - 1; // move hoop to the left
									if (moveUp == 1'b1) 
										ballY <= ballY - 1;
									if (moveDown == 1'b1)
										ballY <= ballY + 1;
									if(hoopX >= 8'd160 & hoopX <= 8'd190)
										case(HoopPos)
											2'b00: hoopY <= 7'd10;
											2'b01: hoopY <= 7'd35;
											2'b10: hoopY <= 7'd60;
											2'b11: hoopY <= 7'd85;
											default: hoopY <= 7'd60;
										endcase
									
									if ((hoopY <= ballY) & (ballY <= (hoopY + 7'd13)) & (hoopX <= 8'd39) & (8'd39 <= (hoopX + 3'd3)))
										scoreEnable <= 1;
									else
										scoreEnable <= 0;
								end
				UpdateWState: denable <= 1'b0;
				
				gameEnd: begin 
					VGAstate <= 2'b11;
					gameStarted <= 1'b0;
				end

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
 
module downCounter(clock, reset, dcount, enable, objectCycles);
    input [0:0] reset;
    input clock;
    reg [25:0] fastcount;
    output reg [1:0] dcount;
    input enable;
	 input [25:0] objectCycles;
    
    always@(posedge clock)
    begin
        if (reset[0] == 0 || enable)
            // Resets downcounter if it gets reset or enabled
            // Counts from 250,000 -> 0. Takes around 5ms, 5ms x 3 = 15ms faster than the refresh rate of the VGA (60Hz = 16.67ms/refresh)
            fastcount <= objectCycles;
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
        defparam U1.n = 7; // number of bits
          
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
        if(ObjToPlot == 1'b1) begin // set variables to player
            VGA_COLOR <= VGA_Player;
            VGA_X <= P_VGA_X;
            VGA_Y <= P_VGA_Y;
        end
        else begin
            VGA_COLOR <= VGA_Hoop; // set variables to hoop
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

module randomCount (Clock, Resetn, E, Q); // random chosen position for hoop
    input Clock, Resetn, E;
    output reg [1:0] Q;
 
    always @ (posedge Clock)
        if (Resetn == 0)
            Q <= 0;
        else if (E)
            Q <= Q + 1'b1;
endmodule
 
module drawBG(plot, CLOCK_50, resetin, VGA_X, VGA_Y, doPlot);
    input plot, resetin, CLOCK_50;
    output doPlot;
    output [7:0] VGA_X;     
    output [6:0] VGA_Y;
    wire Ey; 
    reg CLK;
    wire reset;
    
    assign Ey = (VGA_X == 8'd160);
    assign doPlot = plot;
	 
	 assign reset = plot;
    
    countX C1(CLOCK_50, resetin, reset, plot, VGA_X);
    countY C2(CLOCK_50, resetin, reset, Ey, VGA_Y);
endmodule
 
module countX(clock, resetin, reset, E, Q); // counter for drawing in horizontal direction
    input clock, reset, E, resetin;
    output reg [7:0] Q;
    
    always@(posedge clock)
        if (resetin == 0 | Q == 8'd160 | reset == 0) // checking if key was pressed or if it reaches the horizontal boundary
            Q <= 0;
        else if (E)
            Q <= Q + 1;
endmodule
 
module countY(clock, resetin, reset, E, Q); // counter for drawing in vertical direction
    input clock, reset, E, resetin;
    output reg [6:0] Q;
    
    always@(posedge clock)
        if (resetin == 0 | Q == 7'd120 | reset == 0)
            Q <= 0;
        else if (E)
            Q <= Q + 1;
endmodule

module scoreCounter(CLOCK_50, reset, enable, val);
	input CLOCK_50, enable, reset;
	output reg [5:0] val;
	reg current, prev;
	
	always@(posedge CLOCK_50)
	begin
		prev <= current;
		current <= enable;
		if(reset == 0)
			val <= 0;
		else if(prev == 0 & current == 1) // preventing score incrementing multiple times
			val <= val + 1; // increment score
	end
endmodule


module gameTimer(CLOCK_50, reset, enable, val, gameEnded); // 60 second counter
	input CLOCK_50, reset, enable;
	output reg [5:0] val;
	output reg gameEnded;
	reg [25:0] fastcount;
	
	always@(posedge CLOCK_50)
    begin
        if (reset == 0 | fastcount == 1'b0)
            fastcount <= 26'd50000000;
        else if(enable)
            fastcount <= fastcount - 1'b1;   
    end
    
    always@(posedge CLOCK_50)
    begin
		  if (val == 7'h3F)
			gameEnded <= 0; 
	 
		  if (val == 0)
					gameEnded <= 1;
        if (reset == 0) begin
            val <= 6'd60; // start from 60
				gameEnded <= 0;
			end
        else if (fastcount == 1'b0)
            val <= val - 1;
    end
endmodule

module hex7seg (hex, display);
    input [3:0] hex;
    output [6:0] display;

    reg [6:0] display;

    /*
     *       0  
     *      ---  
     *     |   |
     *    5|   |1
     *     | 6 |
     *      ---  
     *     |   |
     *    4|   |2
     *     |   |
     *      ---  
     *       3  
     */
    always @ (hex)
        case (hex)
            4'h0: display = 7'b1000000;
            4'h1: display = 7'b1111001;
            4'h2: display = 7'b0100100;
            4'h3: display = 7'b0110000;
            4'h4: display = 7'b0011001;
            4'h5: display = 7'b0010010;
            4'h6: display = 7'b0000010;
            4'h7: display = 7'b1111000;
            4'h8: display = 7'b0000000;
            4'h9: display = 7'b0011000;
            4'hA: display = 7'b0001000;
            4'hB: display = 7'b0000011;
            4'hC: display = 7'b1000110;
            4'hD: display = 7'b0100001;
            4'hE: display = 7'b0000110;
            4'hF: display = 7'b0001110;
        endcase
endmodule
