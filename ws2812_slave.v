`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Richard Aplin ( @drtune )
// 
// Create Date:    12:00:01 10/30/2017 
// Design Name: 
// Module Name:    ws2812_slave 
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

//useful : https://cpldcpu.com/2014/01/14/light_ws2812-library-v2-0-part-i-understanding-the-ws2812/

module ws2812_slave(
    input DIN,
	 output reg DOUT,	//..daisy chain :-)
	 input CLK_40,
	 output reg [7:0] rx_data,
	 output reg frame_sync,
	 output reg rx_strobe,
	 input reset
    );
	parameter WIDTH=8;
	parameter STATE_IDLE=0;
	parameter STATE_RX=1;
	parameter LED_STRING_LENGTH=32*36;
	integer i;
	
	parameter WS_CLK_DIV=8;	//8 * 25ns = 200ns base clock for reading WS2182 pixels
	reg [3:0] wsclk_div;
	
	reg [3:0] reset_counter;	//saturates
	parameter RESET_COUNTER_MAX=15;
	parameter RESET_COUNTER_THRESHOLD=2000/200; //2000ns or longer low is a reset
	
	reg [1:0] bit_sample_window;	
	reg sampled_bit;
	parameter BIT_SAMPLE_COUNT=2;	//sample data bit 2 clocks after the rising edge   
	
	reg [WIDTH-1:0] rx_shift;
	reg [1:0] state;
	reg [2:0] bit_count;
	
	initial 
	begin
		state=STATE_IDLE;
		rx_data=0;
		rx_shift=0;
		rx_strobe=0;
		bit_count=0;
	end
	
	// sync to the FPGA clock using a 3-bit shift register
	reg [2:0] DINr;  always @(posedge CLK_40) DINr <= {DINr[1:0], DIN};
	wire DIN_risingedge = (DINr[2:1]==2'b01);  // now we can detect DIN rising edges
	wire DIN_fallingedge = (DINr[2:1]==2'b10);  // and falling edges
	wire DIN_data= DINr[1];
	reg [10:0] byte_count;

	reg wsclk_div_signal=0;
	
	wire passthru_mode=(byte_count==LED_STRING_LENGTH*3);
	wire frame_reset=(reset_counter>RESET_COUNTER_THRESHOLD);
	
	always @(posedge CLK_40 or posedge reset)
	begin
		if (reset)
		begin
			wsclk_div<=0;
		end
		else
		begin
			if (wsclk_div==0)
			begin
				wsclk_div<=WS_CLK_DIV-1;
				wsclk_div_signal<=1;
			end
			else
			begin
				wsclk_div_signal<=0;
				wsclk_div<=wsclk_div-1;
			end
		end
	end
	
	always @(posedge wsclk_div_signal or posedge reset)
	begin
		if (reset)
		begin
			rx_strobe<=0;
			reset_counter<=0;
			DOUT<=1;
			frame_sync<=0;
			bit_count<=0;
			byte_count<=0;
		end
		else
		begin
			rx_strobe<=0;
			
			if (passthru_mode)
			begin
				DOUT<=DIN_data;
			end
			
			if (DIN_fallingedge)
			begin
				reset_counter<=0;
			end
			else
			if (DIN_risingedge)
			begin
				if (frame_reset)
				begin
					//frame reset 
					frame_sync<=1;
					bit_count<=0;
					byte_count<=0;
				end
				else
				if (!passthru_mode)
				begin
					//normal data bit edge (not a reset)
					frame_sync<=0;
					
					rx_shift<={rx_shift[6:0],DIN_data};
					if (bit_count==7)
					begin
						//read byte
						rx_data<=rx_shift;
						rx_strobe<=1;
						bit_count<=0;
						byte_count<=byte_count+1;
					end
					else
						bit_count<=bit_count+1;
					
					//set up sampling of next bit
					bit_sample_window<=BIT_SAMPLE_COUNT+1;	
				end
			end
			else	//not an edge
			if (DIN_data==0)
			begin	//count length of a low on the line to detect resets
				if (reset_counter<RESET_COUNTER_MAX) reset_counter<=reset_counter+1;
			end
			
			if (bit_sample_window)begin
				if (bit_sample_window==1)
					sampled_bit=DIN_data;
				bit_sample_window<=bit_sample_window-1;
			end
		
		end	//not reset
	end

endmodule
