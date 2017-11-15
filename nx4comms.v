`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// OpenNX4 - Open source firmware for Barco NX4 tiles
// Company: Bohemian Bits
// Engineer: Richard Aplin (Twitter: @DrTune)
// Copyright (C) 2017 Richard Aplin
// Released under the Attribution-NonCommercial 2.0 Generic (CC BY-NC 2.0)
// You may not use this code or a derived work for commercial purposes unless you have a commercial license, contact drtune@gmail.com
//////////////////////////////////////////////////////////////////////////////////
// Create Date:    12:59:34 10/25/2017 
// Design Name: 
// Module Name:    nx4comms 
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
// test uart is on Data input port on the NX4; 5 (tx),4,8(rx),7
`include "nx4_header_file.vh"

module nx4comms(
    input serial_clk_in,  		//SPI SCK /I2S PCM_BCLK, I2C SCK (no clock stretching supported/reqd)
    output reg serial_output,	//UART host rx, SPI (MISO), I2S (PCMx_DIN). I2C SDA pulldown if active
    input serial_input,			//WS2812, UART host tx, SPI (MOSI) and I2S (PCMx_DOUT) all use this. I2C SDA if anyone wants to do it
    input serial_input2,		//for 2-bit parallel versions of SPI or I2S
    input serial_select,		//if you want SPI with CS, here's where you'd do it

	 output reg comms_status_led,
	 
	 output com_rx_strobe,	//high for one CLK40 cycle when new data is ready, get it while it's hot!
	 output reg com_rx_start,	//high for same cycle as above if this is the first byte of a new message (which we know for SPI and I2C for example, not UART)
	 output reg com_rx_end_strobe,	//high for one cycle (not same as RX_STROBE) when we 'know' the message is complete. In some cases we don't know (UART) unless we do a timeout or BREAK or something else
	 output reg [7:0] com_rx_data,
	 input 		[7:0] com_tx_data,
	 input 		com_tx_strobe,	//assert for +ve edge of CLK40
	 output reg com_tx_ready,	//high as long as data can be written to the tx (clocked on +ve CLK40)
	 output reg com_reset_cmd_state,
	 
 	 input `NX4_REGISTERS_FLAT, 
	 
	 input reset,
	 input CLK_40
    );

integer unpk_idx;
reg `NX4_REGISTERS_SQUARE;
always @(*) begin
`NX4_REGISTERS_FLAT_TO_SQUARE
end
/*
genvar unpk_idx; 
generate for (unpk_idx=0; unpk_idx<(PK_LEN); unpk_idx=unpk_idx+1) 
begin; 
assign PK_DEST[unpk_idx][((PK_WIDTH)-1):0] = PK_SRC[((PK_WIDTH)*unpk_idx+(PK_WIDTH-1)):((PK_WIDTH)*unpk_idx)]; end; endgenerate;
*/

wire [2:0] com_rx_mode;	//various protocols supported, SPI, UART, I2C etc
assign com_rx_mode=nx4_registers[`OpenNX4_REG_COMCTL][`OpenNX4_REG_COMCTL_BIT_IN_COM_MODE+2 -:3];
wire [2:0] com_tx_mode;	
assign com_tx_mode=nx4_registers[`OpenNX4_REG_COMCTL][`OpenNX4_REG_COMCTL_BIT_OUT_COM_MODE+2 -:3];

/* todo add tx fifo
wire tx_fifo_din;
wire tx_fifo_wr_en;
wire tx_fifo_rd_en;
wire tx_fifo_dout;
wire tx_fifo_full;

comms_tx_fifo tx_fifo (
    .clk(CLK_40), 
    .rst(reset), 
    .din(tx_fifo_din), 
    .wr_en(tx_fifo_wr_en), 
    .rd_en(tx_fifo_rd_en), 
    .dout(tx_fifo_dout), 
    .full(tx_fifo_full)
    //.empty(tx_fifo_empty)
    );
*/


wire spi_cs=serial_select;
wire spi_clk=serial_clk_in;
wire spi_tx_ready;
wire spi_data_out;
wire spi_rx_strobe;
wire [7:0] spi_rx_data;
reg [7:0] spi_tx_data;
wire spi_rx_start;
wire spi_rx_end_strobe;
reg spi_tx_strobe;
    
simple_spi_slave #(.WIDTH(8)) spi (
    .mosi(serial_input),
	 .miso(spi_data_out),
	 .sck(spi_clk),
	 .cs(spi_cs),
	 .CLK_40(CLK_40), 
    .rx_start(spi_rx_start), 
    .rx_end_strobe(spi_rx_end_strobe), 
    .rx_data(spi_rx_data), 
    .rx_data_strobe(spi_rx_strobe),
	 .tx_data(spi_tx_data),
    .tx_ready(spi_tx_ready), 
	 .tx_data_strobe(spi_tx_strobe),
	 .reset(reset)
    );
	 

wire ws2812_data_out;
wire [7:0] ws2812_rx_data;
wire ws2812_rx_strobe;
reg ws2812_rx_start;
//reg [1:0] fb_we_mask;

ws2812_slave ws2812_rx (
    .DIN(serial_input), 
    .DOUT(ws2812_data_out), 
    .CLK_40(CLK_40), 
    .rx_data(ws2812_rx_data), 
    .frame_sync(ws2812_frame_sync), 
    .rx_strobe(ws2812_rx_strobe), 
    .reset(reset)
    );

wire uart_tx;
wire [7:0] uart_rx_data;
wire [8:0] uart_baud_rate;	//347=115kbps  (40000000/baud_rate)
wire uart_rx_data_strobe;
reg [7:0] uart_tx_data;
reg  uart_tx_strobe;
wire uart_rx_break_detect;
wire uart_tx_ready;
//reg uart_rx_start;


uart uart (
    .rx(serial_input), 
    .tx(uart_tx), 
    .rx_data(uart_rx_data), 
    .baud_rate(uart_baud_rate), 
    .rx_data_strobe(uart_rx_data_strobe), 
    .tx_data(uart_tx_data), 
    .tx_data_strobe(uart_tx_strobe), 
    .tx_ready(uart_tx_ready), 
	 .rx_break_detect(uart_rx_break_detect),
    .CLK_40(CLK_40), 
    .reset(reset)
    );



//multiplex the serial comms output between the various protocols depending on what mode we're in
always @(com_rx_mode or spi_data_out or ws2812_data_out or uart_tx) 
begin 
 case (com_rx_mode) // COM_MODE_..
	3'b000   : serial_output <= spi_data_out; 
	3'b001   : serial_output <= spi_data_out; 
	3'b010   : serial_output <= ws2812_data_out; 
	3'b011   : serial_output <= uart_tx; 
	3'b100   : serial_output <= 0; 
	3'b101   : serial_output <= 0; 
	3'b110   : serial_output <= 0; 
	3'b111   : serial_output <= 0; 
 endcase 
end 

//support up to 64k message size
wire [1:0] message_length_prefix;
assign  message_length_prefix = nx4_registers[`OpenNX4_REG_SYSCTL][`OpenNX4_REG_SYSCTL_BIT_MSG_LENGTH_PREFIX1 -:2];
reg [1:0]  message_length_prefix_counter;	//count of loading message_length_count (loaded in reverse)
reg [15:0] message_length_count;	//because uart doesn't have delineation of messages (we support sending a BREAK to resync but it's too slow to do for each command) so UART stuff has a byte prefix 0..255 length. If this byte is zero, the next two bytes are length lsb,msb 

reg rx_strobe;
reg uart_rx_end_strobe;
wire mask_rx_strobe_passthru;
assign mask_rx_strobe_passthru = (message_length_prefix_counter==0);
assign rx_extend_start=0; //mask_rx_strobe_passthru;
reg simulated_end_strobe;
assign com_rx_strobe = rx_strobe & mask_rx_strobe_passthru;
reg uart_rx_start;
				
always @(*)
begin 
 casex (com_rx_mode) 
	3'b00x   : begin  // COM_MODE_..
		rx_strobe = spi_rx_strobe;
		com_rx_data = spi_rx_data;
		//if (`SPI_USES_CS)begin
		//	com_rx_start = spi_rx_start;
		//end
		//if (`SPI_USES_CS)begin
		//	com_rx_end_strobe = spi_rx_end_strobe;
		//end
		com_rx_start = uart_rx_start; //<<use the one generated by the uart controller 
		com_rx_end_strobe = uart_rx_end_strobe;	//same use uart
		com_tx_ready=spi_tx_ready;
		spi_tx_strobe=com_tx_strobe;
		spi_tx_data=com_tx_data;
		uart_tx_strobe=0;
		uart_tx_data=0;
		end
	3'b010   : begin
		rx_strobe = ws2812_rx_strobe; 
		com_rx_data = ws2812_rx_data;
		com_rx_start = ws2812_rx_start;
		com_tx_ready=0;
		com_rx_end_strobe = uart_rx_end_strobe;
		uart_tx_strobe=0;
		uart_tx_data=0;
		spi_tx_strobe=0;
		spi_tx_data=0;
		end
	3'b011   : begin
		rx_strobe = uart_rx_data_strobe & mask_rx_strobe_passthru; 
		com_rx_data = uart_rx_data;
		com_tx_ready=uart_tx_ready;
		com_rx_start=uart_rx_start;
		com_rx_end_strobe = uart_rx_end_strobe;
		uart_tx_strobe=com_tx_strobe;
		uart_tx_data=com_tx_data;
		spi_tx_strobe=0;
		spi_tx_data=0;
		end
	default:	begin
		rx_strobe = 0; 
		com_rx_data = 0;
		com_tx_ready=0;
		com_rx_start = 0;
		com_rx_end_strobe = 0;
		spi_tx_strobe=0;
		spi_tx_data=0;
		uart_tx_strobe=0;
		uart_tx_data=0;
		end
	//3'b100   : 
	//3'b101   : 
	//3'b110   : 
	//3'b111   : 
 endcase 
end 

wire [`OpenNX4_REG_IOCTL_BIT_UART_BAUD_BITCOUNT-1:0] baud_multiplier;
assign baud_multiplier=nx4_registers[`OpenNX4_REG_IOCTL][(`OpenNX4_REG_IOCTL_BIT_UART_BAUD_0+`OpenNX4_REG_IOCTL_BIT_UART_BAUD_BITCOUNT)-1 -:`OpenNX4_REG_IOCTL_BIT_UART_BAUD_BITCOUNT ];
assign uart_baud_rate={0,0,0,0,0,0,0,0,(40000000/`BASELINE_UART_BAUD) }>>baud_multiplier;		

always @(posedge CLK_40 or posedge reset)
begin
	if (reset)begin
		comms_status_led<=0;
		ws2812_rx_start<=0;
		simulated_end_strobe<=0;
		uart_rx_end_strobe<=0;
		com_reset_cmd_state<=1;
	end
	else
	begin
		if (com_rx_strobe)begin
			comms_status_led<=~comms_status_led;
		end

		if (com_reset_cmd_state)begin
			message_length_prefix_counter<=message_length_prefix;
			message_length_count<=0;
			com_reset_cmd_state<=0;
		end
		
		uart_rx_end_strobe<=0;
		ws2812_rx_start<=0;

		//figure out the start/end of messages
		case (com_rx_mode)
			`COM_MODE_SPI_2BIT: //todo
				begin
				end
			`COM_MODE_SPI: 
				begin
				end
				
			`COM_MODE_WS2812: 
				begin
					if (ws2812_frame_sync)
					begin
						uart_rx_end_strobe<=1;
						ws2812_rx_start<=1;
					end
				end
			`COM_MODE_UART: 
				begin
					
					// *** Test - echo everything back, this will mess with other things 
//`define UART_LOOPBACK_TEST					
`ifdef UART_LOOPBACK_TEST
					uart_tx_strobe<=0;
					if (uart_rx_data_strobe)
					begin
						uart_tx_data<=uart_rx_data;
						uart_tx_strobe<=1;
					end
`endif	
/*
`define UART_BOOTUP_MESSAGE
`ifdef UART_BOOTUP_MESSAGE
					//optional bootup message sent over uart
					if (uart_tx_ready)
					begin
						//Send one-time bootup message to say howdy
						if (boot_message_pos>0 && !uart_tx_strobe)
						begin: bootmsg
							uart_tx_data<=boot_message[(boot_message_pos*8) -:8]; 
							uart_tx_strobe<=1;
							boot_message_pos<=boot_message_pos-1;
						end
					end
`endif
*/

					//send a BREAK to reset the UART to default baud rate and message length prefix
					if (uart_rx_break_detect)
					begin
						//uart_rx_start<=1;
						com_reset_cmd_state<=1;
					end
					
				end
			`COM_MODE_I2C: 
				begin
				end
		endcase
			
			
		//handle message length prefixes for UART and SPI if you're not using CS
		if (com_rx_mode==`COM_MODE_UART || (com_rx_mode==`COM_MODE_SPI && !`SPI_USES_CS))begin
			if (uart_rx_data_strobe || spi_rx_strobe)
			begin
				//incoming byte..
				//are we expecting a length prefix?
				if (message_length_prefix)begin
					case (message_length_prefix_counter)
						0: begin	//due to switches elsewhere the rx_strobe will be passed through if case 0, else it will be masked from being output
								if (message_length_count!=0)begin
									uart_rx_start<=0;
									message_length_count<=message_length_count-1;
									if (message_length_count==1)begin
										uart_rx_end_strobe<=1;
									end
								end
								else
								begin
									message_length_prefix_counter<=message_length_prefix;	//next byte is another length
									uart_rx_start<=1;
								end
							end
						1:	begin	//rx_data_strobe output will be suppressed
								message_length_count[7:0]<=com_rx_data;
								message_length_prefix_counter<=0;
								uart_rx_start<=1;
							end
						2: begin //rx_data_strobe output will be suppressed
								message_length_count[15:8]<=com_rx_data;
								message_length_prefix_counter<=1;
							end
						default:
							message_length_prefix_counter<=0;
					endcase
				end
				else 	//if zero message_length_prefix, take care of the housekeeping
				begin
					if (uart_rx_data_strobe)
					begin
						uart_rx_start<=0;
					end
				end
				
			end //end handling incoming byte 
			
			
		end
		
	end //not reset
	
end

/*
always @(message_length_prefix)
begin
	message_length_prefix_counter<=message_length_prefix;
end
*/

endmodule
