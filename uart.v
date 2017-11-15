`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:  Richard Aplin ( @drtune )
// 
// Create Date:    17:37:18 11/01/2017 
// Design Name: 
// Module Name:    uart 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module uart(
	input rx,
	output reg tx,
	output reg [7:0] rx_data,
	output reg rx_data_strobe,	//strobes for one clk40
	output reg rx_break_detect,
	input [8:0] baud_rate,
	input [7:0] tx_data,
	input tx_data_strobe,
	output reg tx_ready,
	input CLK_40,
	input reset
    );
parameter BAUD_WIDTH=9; //add more bits if you want lower baud rates
parameter TX_FIFO_LENGTH=8;

parameter RX_STATE_IDLE=0;
parameter RX_STATE_RX=1;

parameter TX_STATE_IDLE=0;
parameter TX_STATE_TX=1;

//reg [7:0] fifo[0:TX_FIFO_LENGTH];
reg [7:0] tx_data_buffer;
reg [7:0] tx_data_output_buffer;
reg [1:0] sync_din;
reg [3:0] rx_bit_count=0;
reg [3:0] tx_bit_count=0;
reg [BAUD_WIDTH:0] rx_baud_counter;	//could probably use a pre-divided clock and save a few bits on these but whatev
reg [BAUD_WIDTH:0] tx_baud_counter;
reg rx_state;
reg tx_state;
reg tx_data_ready;
reg tx_data_loaded;
parameter BREAK_DETECT_BIT_COUNT=9;	//"There are short breaks and long breaks. In general a short break is one that lasts longer than one frame time, but less than two frame times. A long break can last two or more frame times. Most serial devices that support breaks use a short break."
reg [3:0] break_detect_counter;
reg rx_framing_error;

always @(posedge CLK_40 or posedge reset)
begin
	if (reset)
	begin
		rx_bit_count<=0;
		rx_data_strobe<=0;
		tx<=1;
		rx_data<=0;
		rx_break_detect<=0;
		break_detect_counter<=0;
		rx_baud_counter<=baud_rate;
		tx_baud_counter<=baud_rate;
		tx_data_loaded<=0;
		rx_state<=RX_STATE_IDLE;
		tx_state<=TX_STATE_IDLE;
		tx_ready<=0;
		tx_data_ready<=0;
		rx_framing_error<=0;
	end
	else
	begin
		//we could low-pass filter the input rx line here if we wanted to, we're sampling at 40mhz 
		sync_din[0]<=rx;
		sync_din[1]<=sync_din[0];
		
		rx_data_strobe<=0;

		case(rx_state)
			RX_STATE_IDLE:
				begin
					if (sync_din[1]==1 && sync_din[0]==0)	//falling edge = start bit
					begin
						break_detect_counter<=0;
						rx_baud_counter<=baud_rate+(baud_rate>>1);	//sample next bit in 1.5 bit times
						rx_bit_count<=0;	//includes stop bit
						rx_state<=RX_STATE_RX;
						rx_framing_error<=0;
					end
				end	//end idle state
				
			RX_STATE_RX:
				begin
					if (rx_baud_counter==0)
					begin
						rx_baud_counter<=baud_rate;

						//check for break condition
						if (sync_din[0]==0)
						begin
							if (break_detect_counter==BREAK_DETECT_BIT_COUNT)	//15 consective bit periods of zeros = a break condition
							begin
								rx_break_detect<=1;
								rx_bit_count<=0;
							end
							else
							begin
								break_detect_counter<=break_detect_counter+1;
							end
						end
						else
						begin	//line is high 
							break_detect_counter<=0;
						end
						
						if (rx_break_detect)
						begin
							if (sync_din[0]==1)	//end of break
							begin
								rx_break_detect<=0;
								rx_state<=RX_STATE_IDLE;
							end
						end
						else //not in break condition
						begin
							
							
							if (rx_bit_count==8)	//stop bit
							begin
								//check stop bit is there
								if (sync_din[1]!=1)
								begin
									//framing error! we may also be in the middle of detecting a break condition, so we stay in the receive 
									//state until a 1 turns up
									rx_framing_error<=1;
								end
								else
								begin
									if (rx_framing_error==0)begin
										rx_data_strobe<=1; //received ok
									end
									rx_state<=RX_STATE_IDLE;
								end
								
							end
							else
							begin
								rx_data[rx_bit_count]<=sync_din[1];
								rx_bit_count<=rx_bit_count+1;
							end
						end
					end
					else
					begin
						rx_baud_counter<=rx_baud_counter-1;
					end
				end	//end rx state
		endcase
		
		if (tx_data_strobe && !tx_data_ready)
		begin
			//load data
			tx_data_buffer<=tx_data;
			tx_data_ready<=1;
			tx_ready<=0;
		end 

		
		if (tx_baud_counter==0)
		begin
			tx_baud_counter<=baud_rate;
			case(tx_state)
				TX_STATE_IDLE:
					begin
						if (tx_data_ready)
						begin
							tx_data_ready<=0;
							tx_data_output_buffer<=tx_data_buffer;
							tx_state<=TX_STATE_TX;
							tx_bit_count<=0;
							//send start bit (0) now
							tx<=0;
						end 
						else
						begin
							tx<=1;	//tx/rx idle state is high
							tx_ready<=1;
						end
					end
				TX_STATE_TX:
					begin
						tx_data_loaded<=0;
							
						if (tx_bit_count==8)	
						begin
							tx<=1;	//stop bit
							tx_state<=TX_STATE_IDLE;
						end
						else
						begin
							tx<=tx_data_output_buffer[tx_bit_count];
							tx_bit_count<=tx_bit_count+1;
						end
					end
			endcase
		end
		else
		begin
			tx_baud_counter<=tx_baud_counter-1;
		end
		//end tx
	end
end

endmodule
