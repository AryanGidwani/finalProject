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
	 start

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

output start; //Start game signal
Hex_To_Letter Segment0 (CLOCK_50, last_data_received, HEX0, start);


endmodule

module Hex_To_Letter(clock, last_data_received, out, start);
	input clock;
	input wire [7:0] last_data_received;
	output reg [6:0] out;
	output reg start;

	parameter [7:0] W = 7'h1D, A = 7'h1C, S = 7'h1B, D = 7'h23, P = 7'h4D;
	always@(posedge clock) begin
		case(last_data_received)
			W: out <= 7'b0000000;
			A: out <= 7'b0001000;
			S: out <= 7'b0010010;
			D: out <= 7'b0100001;
			P: start <= 1;
			default: start <= 0;
		endcase
	end



endmodule
