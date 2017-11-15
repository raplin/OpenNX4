`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Richard Aplin ( @drtune )
// 
// Create Date:    20:11:02 10/27/2017 
// Design Name: 
// Module Name:    simple_spi_slave 
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


module simple_spi_slave #(	parameter WIDTH=8) (
    input mosi,
	 output reg miso,
    input sck,
	 input cs,
	 input CLK_40,
	 output reg [WIDTH-1:0] rx_data,
	 input [WIDTH-1:0] tx_data,
	 output reg tx_ready,	//valid for one clk40 
	 input tx_data_strobe,
	 output reg rx_data_strobe,
	 output reg rx_start,	//during the same clk40 as rx_data_strobe, but only the first one of each 'transaction' (definition depends on wire protocol)
	 output reg rx_end_strobe,
	 
	 input reset
    );
	parameter STATE_IDLE=0;
	parameter STATE_RX=1;
	integer i;
	
	reg [WIDTH-1:0] tx_shift;
	reg [WIDTH-1:0] tx_buf;
	reg [1:0] state;
	reg [2:0] bit_count;
	
	initial 
	begin
		state=STATE_IDLE;
		rx_data=0;
		tx_shift=0;
		bit_count=0;
	end
	
	//assign miso=0;

	// sync SCK to the FPGA clock using a 3-bits shift register
	reg [2:0] SCKr;  always @(posedge CLK_40) SCKr <= {SCKr[1:0], sck};
	wire SCK_risingedge = (SCKr[2:1]==2'b01);  // now we can detect SCK rising edges
	wire SCK_fallingedge = (SCKr[2:1]==2'b10);  // and falling edges
	wire SCK_data= SCKr[1];

	// and for MOSI
	reg [2:0] MOSIr;  always @(posedge CLK_40) MOSIr <= {MOSIr[1:0], mosi};
	wire MOSI_data = MOSIr[1];
	wire MOSI_fallingedge = (MOSIr[2:1]==2'b10); 
	
	//and cs
	reg [2:0] CSr;  always @(posedge CLK_40) CSr <= {CSr[1:0], cs};
	wire CS_data = CSr[1];
	wire CS_fallingedge = (CSr[2:1]==2'b10); 
	wire CS_risingedge = (CSr[2:1]==2'b01); 
	
		
	always @(posedge CLK_40 or posedge reset)
	begin
		if (reset)
		begin
			state<=STATE_IDLE;
			rx_data_strobe<=0;
			miso<=0;
			tx_ready<=1;
			bit_count<=0;
			rx_start<=0;
			rx_end_strobe<=0;
		end
		else
		begin
			if (rx_data_strobe)
			begin
				rx_data_strobe<=0;
				rx_start<=0;
			end

			if (tx_data_strobe)
			begin
				tx_buf<=tx_data;
				tx_ready<=0;
			end

			rx_end_strobe<=0;

			if (CS_fallingedge)
			begin
				//start of message
				bit_count<=0;
				tx_shift<=tx_buf;	//should set the first MISO (tx) byte before the master drops CS; usually this byte is a 'don't care' - we could force it to be zero here optionally
				rx_start<=1;
			end
			else
			if (CS_risingedge)
			begin
				rx_end_strobe<=1;
			end
			else
			begin
				//ok we basically do MSB first, CPHA=1  (change data on rising SCLK and sample on falling), CS=active low
				if (SCK_fallingedge)
				begin
					//shift in msb first
					rx_data<={ rx_data[WIDTH-2:0],MOSI_data } ;
					bit_count<=bit_count+1;
					if (bit_count==7)	//bit count will wrap automatically
					begin
						rx_data_strobe<=1;
					end
				end
				else
				if (SCK_risingedge)
				begin
					miso<=tx_shift[ 7-bit_count];
					if (bit_count==7)
					begin
						if (tx_ready)
						begin
							//underflow on tx data, flag it if you want
						end
						tx_shift<=tx_buf;
						tx_ready<=1;
					end
				end
					/*
					case (state)
					STATE_IDLE: begin
										if (rx_shift == SYNC_WORD) 
										begin
											state<=STATE_RX;
											bit_count<=0;
										end
									end
					STATE_RX:	begin
										if (bit_count==WIDTH-1)
										begin
											rx_data<=rx_shift;
											rx_ready<=1;
											state<=STATE_IDLE;
										end
										else
											bit_count<=bit_count+1;
									end
					default:		begin
									end
					endcase
				end
				*/
			end
		end	//not reset
	end

endmodule
