`default_nettype none

module PS2_Comm(
    // inputs
    CLOCK_50,
    KEY,

    // bidrectionals
    PS2_CLK,
    PS2_DAT,

    // outputs
	 start,
	 moveUp,
	 moveDown

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
reg [7:0] prev_data_received;
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
		prev_data_received <= 8'h00;
		last_data_received <= 8'h00;
		counter <= 0;
	end
		
	else if (ps2_key_pressed == 1'b1) begin
		prev_data_received <= last_data_received;
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
output moveUp, moveDown;

wire halfClk;
Hex_To_Letter Segment0 (CLOCK_50, last_data_received, start);
PS2FSM FSM(CLOCK_50, KEY[0], last_data_received, prev_data_received, moveUp, moveDown);
clockCounter halfclock(CLOCK_50, KEY[0], halfClk);

endmodule

module Hex_To_Letter(clock, last_data_received, start);
	input clock;
	input wire [7:0] last_data_received;
	output reg start;

	parameter [7:0] W = 8'h1D, A = 8'h1C, S = 8'h1B, D = 8'h23, P = 8'h4D, Space = 8'h29;
	always@(posedge clock) begin
		case(last_data_received)
			P: start <= 1;
			default: begin 
			start <= 0;
			end
		endcase
	end

endmodule

module PS2FSM(clock, reset, last_data_received, prev_data_received, moveUp, moveDown);
	input clock, reset;
	input wire [7:0] last_data_received, prev_data_received;
	output reg moveUp, moveDown;
	reg [3:0] currentState, nextState;
	parameter [1:0] S1 = 2'b00, S2 = 2'b01, S3 = 2'b10, S4 = 2'b11; 
	parameter [7:0] W = 8'h1D, A = 8'h1C, S = 8'h1B, D = 8'h23, P = 8'h4D, F = 8'hF0;
	
	always@ (posedge clock)
		begin: state_table
		case (currentState)
			S1: if((prev_data_received == W | prev_data_received == P | prev_data_received == S) & last_data_received == W) 
					nextState <= S2; // go to next state
				 else if ((prev_data_received == W | prev_data_received == S | prev_data_received == P) & last_data_received == S)
					nextState <= S3;
				 else 
					nextState <= S1;
					
			S2: if(last_data_received == F) 
					nextState <= S4;
				 else 
					nextState <= S2;
					
			S3: if (last_data_received == F)
					nextState <= S4;
				 else
					nextState <= S3;
			
			S4: nextState <= S1;

			default: nextState <= S1;
		endcase
		end
		
		always @(posedge clock)
		begin: state_FFs
			if (!reset)
				currentState <= S1;
			else
				currentState <= nextState;
		end
		
		always@(posedge clock)
		begin 
		case (currentState)
			S1: begin
					moveUp <= 0;
					moveDown <= 0;
				 end
			S2: moveUp <= 1; // update signal in the state
			S3: moveDown <= 1; // sending correct signal
		endcase
		end
endmodule

module clockCounter(clock, reset, count);
    input [0:0] reset;
    input clock;
    reg [0:0] fastcount;
    output reg [0:0] count;
    wire enable;
    
    assign enable = (fastcount == 0) ? 1 : 0; 
    always@(posedge clock)
    begin
        if (reset[0] == 0)
            // reset to initial image with 
            fastcount <= 0;
        else 
            fastcount <= fastcount + 1'b1;
            
    end
    
    always@(posedge clock)
    begin
        if (reset[0] == 0)
            count <= 0;
        else if (enable == 1)
            count <= count + 1;
    end
endmodule
