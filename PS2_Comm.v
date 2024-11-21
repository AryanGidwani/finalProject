`default_nettype none

module PS2_Comm(
    // inputs
    CLOCK_50,
    KEY,

    // bidrectionals
    PS2_CLK,
    PS2_DAT,

    // outputs
    HEX0,
    HEX1,
	 start,
	 moveUp

);


/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/


/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/

// Inputs
input	wire			CLOCK_50;
input	wire 	[0:0]	KEY;        // KEY[0] reset

// Bidirectionals
inout	wire			PS2_CLK;
inout	wire			PS2_DAT;


// Outputs
output	wire	[6:0]	HEX0;
output	wire 	[6:0]	HEX1;
assign HEX1 = 7'b1111111;

/*****************************************************************************
 *                 Internal Wires and Registers Declarations                 *
 *****************************************************************************/

// Internal Wires
wire		[7:0]	ps2_key_data;
wire				ps2_key_pressed;
wire           error_communication;
wire command_was_sent;

// Internal Registers
reg [7:0] last_data_received;
reg [2:0] counter;
// State Machine Registers

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/

/*****************************************************************************
 *                             Sequential Logic                              *
 *****************************************************************************/

/*
*  receiving data
*/
always @(posedge CLOCK_50)
begin
	if (KEY[0] == 1'b0) begin
		last_data_received <= 8'h00;
		counter <= 0;
	end
		
	else if (ps2_key_pressed == 1'b1) begin
		last_data_received <= ps2_key_data;
			
		if(counter == 3'd6)
			counter <= 1'b0;
		else
			counter <= counter + 1'b1;		
	end
end

/*
*  sending data
*/

/*****************************************************************************
 *                            Combinational Logic                            *
 *****************************************************************************/

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

PS2_Controller PS2 (
	// Inputs
	.CLOCK_50			(CLOCK_50),
	.reset				(~KEY[0]),
    .the_command        (0),
    .send_command       (0),

	// Bidirectionals
	.PS2_CLK			(PS2_CLK),
 	.PS2_DAT			(PS2_DAT),

	// Outputs
    .command_was_sent   (command_was_sent),
    .error_communication_timed_out (error_communication),
	.received_data		(ps2_key_data),
	.received_data_en	(ps2_key_pressed)
);

output start, moveUp; //Start game signal
Hex_To_Letter Segment0 (CLOCK_50, last_data_received, HEX0, start, moveUp);


endmodule

module Hex_To_Letter(clock, last_data_received, out, start);
	input clock;
	input wire [7:0] last_data_received;
	output reg [6:0] out;
	output reg start;

	parameter [7:0] W = 8'h1D, A = 8'h1C, S = 8'h1B, D = 8'h23, P = 8'h4D;
	always@(posedge clock) begin
		case(last_data_received)
			W: out <= 7'b0000000;
			A: out <= 7'b0001000;
			S: out <= 7'b0010010;
			D: out <= 7'b0100001;
			P: start <= 1;
			Space: moveUp <= 1;
			default: begin 
			start <= 0;
			end
		endcase
	end



endmodule

module PS2FSM(clock, last_data_received, out, moveUp);
	input clock;
	input wire [7:0] last_data_received;
	output reg [6:0] out;
	output reg moveUp;
	reg [3:0] currentState, nextState;
	parameter [3:0] S1 = 4'b0000, S2 = 4'b0001, S3 = 4'b0010, S4 = 4'b0011, S5 = 4'b0100; 
	parameter [7:0] W = 8'h1D, A = 8'h1C, S = 8'h1B, D = 8'h23, P = 8'h4D, Space = 8'h29, F = 8'hF0;
	
	always@ (posedge clock)
		begin: state_table
		case (currentState)
			S1: if(last_data_received == Space) 
					nextState <= S2;
				 else if(last_data_received == F)
					nextState <= S4;
					
			S2: if(last_data_received == Space) 
					nextState <= S2;
				 else if(last_data_received == F)
					nextState <= S3;
					
			S3: if(last_data_received == Space) 
					nextState <= S5;
				 else if(last_data_received == F)
					nextState <= S3;
			
			S4: if(last_data_received == Space) 
					nextState <= S5;
				 else if(last_data_received == F)
					nextState <= S4;
			
			S5: if(last_data_received == Space) 
					nextState <= S2;
				 else if(last_data_received == F)
					nextState <= S3;
			default: nextState <= S1;
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
			S1: moveUp <= 0;
			S3: moveUp <= 1;
			S5: moveUp <= 0;
		endcase
		end
endmodule
