`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// OpenNX4 - Open source firmware for Barco NX4 tiles
// Company: Bohemian Bits
// Engineer: Richard Aplin (Twitter: @DrTune)
// Copyright (C) 2017 Richard Aplin
// Released under the Attribution-NonCommercial 2.0 Generic (CC BY-NC 2.0)
// You may not use this code or a derived work for commercial purposes unless you have a commercial license, contact drtune@gmail.com
//////////////////////////////////////////////////////////////////////////////////
// Create Date:    00:51:19 11/03/2017 
// Design Name: 
// Module Name:    command_processor 
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
`include "nx4_header_file.vh"

module command_processor(
    //intensity lookup table (12/16 bit, 256 entries)
	 output reg il_wea,	//write enable
	 output [`INTERNAL_PIXEL_WIDTH_BITS-1:0] il_din,	//write data
	 output [7:0] il_addr,	//address
	 
	 //framebuffers
	 output [`FB_ADDR_WIDTH-1:0] fb_addr,	//shared address bus
	 output [7:0] fb_dout,		//and data
	 output fb0_we,	//separate WE for each fb
	 output fb1_we,

	 output status_led_red,

	 output `NX4_REGISTERS_FLAT, 
	 
	 input CLK_40,
	 input vsync,
	 input reset,
	 input [15:0] frame_count,
	 
	 input com_rx_strobe,	//high for one CLK40 cycle when new data is ready, get it while it's hot!
	 input com_rx_start,	//high for same cycle as above if this is the first byte of a new message (which we know for SPI and I2C for example, not UART)
	 input com_rx_end_strobe,	//high for one cycle (not same as RX_STROBE) when we 'know' the message is complete. In some cases we don't know (UART) unless we do a timeout or BREAK or something else
	 input [7:0] com_rx,
	 output reg [7:0] com_tx,
	 output reg com_tx_strobe,	//assert for +ve edge of CLK40
	 input com_tx_ready,	//high as long as data can be written to the tx (clocked on +ve CLK40)
	 input com_reset_cmd_state,
	 
	 output reg cp_mem_sram_op_req_strobe,
	 output reg cp_mem_flash_op_req_strobe,
	 output reg [7:0] cp_mem_dout,
	 input [7:0] cp_mem_din,
	 output reg cp_mem_we,
	
	 output reg [15:0] cp_command_addr,	//any more bits beyond this (for addressing flash etc) are wired into other registers (paged)
		
	 input [15:0] nx4_status,
		
	 input [7:0] reg_read_bus,
	 output reg reg_read_strobe
	 );


	//boot message (goes over uart)
	parameter BOOT_MESSAGE_LENGTH=16;

	reg [4:0] boot_message_pos;
	//Test is stored MSB:LSB order so output (in this case with 16-char message) bits [128:120] first
	reg [8*BOOT_MESSAGE_LENGTH:1] boot_message= "\nOpenNX4 V0.0.1\n"; 

	reg [7:0] cmd_modulo_counter;
	reg `NX4_REGISTERS_SQUARE; 

	//ridiculous syntax stuff just so we can hand a bank of registers between verilog modules. Achieves precisely nothing in terms of chip logic.
	assign nx4_registers_flat[((16<<3)-1) -:8] = nx4_registers[15][7:0];
	assign nx4_registers_flat[((15<<3)-1) -:8] = nx4_registers[14][7:0];
	assign nx4_registers_flat[((14<<3)-1) -:8] = nx4_registers[13][7:0];
	assign nx4_registers_flat[((13<<3)-1) -:8] = nx4_registers[12][7:0];
	assign nx4_registers_flat[((12<<3)-1) -:8] = nx4_registers[11][7:0];
	assign nx4_registers_flat[((11<<3)-1) -:8] = nx4_registers[10][7:0];
	assign nx4_registers_flat[((10<<3)-1) -:8] = nx4_registers[9][7:0];
	assign nx4_registers_flat[((9<<3)-1) -:8] = nx4_registers[8][7:0];
	assign nx4_registers_flat[((8<<3)-1) -:8] = nx4_registers[7][7:0];
	assign nx4_registers_flat[((7<<3)-1) -:8] = nx4_registers[6][7:0];
	assign nx4_registers_flat[((6<<3)-1) -:8] = nx4_registers[5][7:0];
	assign nx4_registers_flat[((5<<3)-1) -:8] = nx4_registers[4][7:0];
	assign nx4_registers_flat[((4<<3)-1) -:8] = nx4_registers[3][7:0];
	assign nx4_registers_flat[((3<<3)-1) -:8] = nx4_registers[2][7:0];
	assign nx4_registers_flat[((2<<3)-1) -:8] = nx4_registers[1][7:0];
	assign nx4_registers_flat[((1<<3)-1) -:8] = nx4_registers[0][7:0];
	
	
	reg last_vsync;
	wire vsync_edge=(vsync && !last_vsync);


	assign status_led_red=nx4_registers[`OpenNX4_REG_IOCTL][`OpenNX4_REG_IOCTL_BIT_RED_LED];
	//todo assign status_led_red=nx4_registers[`OpenNX4_REG_IOCTL][`OpenNX4_REG_IOCTL_BIT_YELLOW_LED];
	//todo assign status_led_red=nx4_registers[`OpenNX4_REG_IOCTL][`OpenNX4_REG_IOCTL_BIT_AMBER_LED];

	reg fb_write_mask;
	reg fb_toggle_strobe;
	reg fb_toggle_state;
	assign fb0_we=(nx4_registers[`OpenNX4_REG_SYSCTL][`OpenNX4_REG_SYSCTL_BIT_FB0_WRITE]^fb_toggle_state) && fb_write_mask && com_rx_strobe;
	assign fb1_we=(nx4_registers[`OpenNX4_REG_SYSCTL][`OpenNX4_REG_SYSCTL_BIT_FB1_WRITE]^fb_toggle_state) && fb_write_mask && com_rx_strobe;

	
	reg cmd_data_first_byte_flag;
	reg [3:0] cmd_state;
	reg [7:0] rx_id_byte;
	wire [3:0] self_unit_id=nx4_registers[`OpenNX4_REG_UNIT_ID];
	
	reg comms_data_in;	
	
	integer next_bb,i;
	integer sign,fb_blend_delta,nextbb;
	
	reg [1:0] cp_addr_inc_delay;	//delay before auto-incrementing address, used to account for extra pipeline stage to flash/sram
	
	assign fb_addr=cp_command_addr;
	assign fb_dout=com_rx;
	
	assign il_addr=cp_command_addr>>1;
	reg [3:0] il_msb_latch;
	assign il_din={{il_msb_latch[3:0]},{com_rx[7:0]}};
	
	assign com_rx_mode=nx4_registers[`OpenNX4_REG_COMCTL][`OpenNX4_REG_COMCTL_BIT_IN_COM_MODE+2 -:3];
	reg is_write;

	wire [2:0] rx_command_mode=rx_id_byte[`CP_CMD_MODE_BIT+2 -:3];

	always @(posedge CLK_40 or posedge reset)
	begin
		if (reset || com_reset_cmd_state) begin
			nx4_registers[`OpenNX4_REG_SYSCTL]<=(1<<`OpenNX4_REG_SYSCTL_BIT_MSG_LENGTH_PREFIX0)|(1<<`OpenNX4_REG_SYSCTL_BIT_AUTO_BUFFER_FLIP)|(1<<`OpenNX4_REG_SYSCTL_BIT_FB0_WRITE);	
			nx4_registers[`OpenNX4_REG_IOCTL]<=(1<<`OpenNX4_REG_IOCTL_BIT_I2C_SDA)|(1<<`OpenNX4_REG_IOCTL_BIT_I2C_SCL) | (`DEFAULT_BAUD_MULTIPLIER<<`OpenNX4_REG_IOCTL_BIT_UART_BAUD_0);
			boot_message_pos<=BOOT_MESSAGE_LENGTH;
		end
		if (reset)begin
			for(i=0;i<`OpenNX4_REG_COUNT;i=i+1) nx4_registers[i]<=0;
			
			nx4_registers[`OpenNX4_REG_FB0_INTENSITY]<='hff;
			nx4_registers[`OpenNX4_REG_FB1_INTENSITY]<=0;
//			nx4_registers[`OpenNX4_REG_FB_BLEND]<='h0;
//			nx4_registers[`OpenNX4_REG_FB_BLEND_STEP]<=5;
//			nx4_registers[`OpenNX4_REG_FB_BLEND_TARGET]<='h0;
			nx4_registers[`OpenNX4_REG_DOT_CORRECT_TEST]<='h07;	//6 bit value, let's keep it low to avoid blowing anything if not row scanning
			nx4_registers[`OpenNX4_REG_COMCTL]<=(`DEFAULT_IN_COM_MODE<<`OpenNX4_REG_COMCTL_BIT_IN_COM_MODE)|(`DEFAULT_OUT_COM_MODE<<`OpenNX4_REG_COMCTL_BIT_OUT_COM_MODE);
			nx4_registers[`OpenNX4_REG_DRIVECTL]<=(`DEFAULT_PIXEL_CLOCK_DIVIDER<<`OpenNX4_REG_DRIVECTL_BIT_PCLK_DIV0)|(`DEFAULT_GSPWM_CLOCK_DIVIDER<<`OpenNX4_REG_DRIVECTL_BIT_BCLK_DIV0);

			fb_write_mask<=0;
			il_msb_latch<=0;
			fb_toggle_state<=0;
			last_vsync<=0;
			fb_toggle_strobe<=0;
			cmd_data_first_byte_flag<=0;
			//self_unit_id<=0;
			cmd_state<=`CP_STATE_IDLE;

			cp_addr_inc_delay<=0;
			com_tx_strobe<=0;
			cp_mem_dout<=0;
			cp_mem_we<=0;
			cp_mem_sram_op_req_strobe<=0;
			cp_mem_flash_op_req_strobe<=0;
			cmd_modulo_counter<=0;
		end
		else
		begin
			is_write = rx_id_byte[`CP_CMD_MODE_BIT_WRITE];
	
`ifdef TURNED_OFF			
			if (com_rx_end_strobe)begin
				//if just finishing a write to the FB, if auto buffer flip is enabled, do it
				if (rx_command_mode==`CP_CMD_MODE_FB && is_write)begin
					if (nx4_registers[`OpenNX4_REG_SYSCTL][`OpenNX4_REG_SYSCTL_BIT_AUTO_BUFFER_FLIP])begin
						fb_toggle_strobe<=1;	//inverts polarity of we bits
					end
				end
			end
			
			if (fb_toggle_strobe)
			begin//trigger automatic fade to other buffer
				fb_toggle_strobe<=0;
				fb_toggle_state<=~fb_toggle_state;
				if (fb_toggle_state)begin
					nx4_registers[`OpenNX4_REG_FB_BLEND_TARGET] <= 'h0;	//blend to fb0
				end
				else
				begin
					nx4_registers[`OpenNX4_REG_FB_BLEND_TARGET] <= 'hff;	//blend to fb1
				end
			end
`endif

			//every vsync transition the buffer fade from where it is now to the target at the desired speed
			last_vsync<=vsync;
			if (vsync_edge)
			begin
				//put stuff here that happens each vsync (6 row scans)
			
`ifdef TURNED_OFF			
				//fade
				fb_blend_delta=(nx4_registers[`OpenNX4_REG_FB_BLEND_TARGET] - nx4_registers[`OpenNX4_REG_FB_BLEND]);
				if (fb_blend_delta)
				begin
					sign=fb_blend_delta[8];
					if (sign)
						next_bb = nx4_registers[`OpenNX4_REG_FB_BLEND] - nx4_registers[`OpenNX4_REG_FB_BLEND_STEP];
					else
						next_bb = nx4_registers[`OpenNX4_REG_FB_BLEND] + nx4_registers[`OpenNX4_REG_FB_BLEND_STEP];

					if ( (((nx4_registers[`OpenNX4_REG_FB_BLEND_TARGET] - next_bb)>>8)&1)^sign )
						nx4_registers[`OpenNX4_REG_FB_BLEND]<=nx4_registers[`OpenNX4_REG_FB_BLEND_TARGET];
					else
						nx4_registers[`OpenNX4_REG_FB_BLEND]<=next_bb;
				end
`endif			
			end
			
			//reset any of the writes that just happened
			il_wea<=0;
			com_tx_strobe<=0;
			//reset some other strobes
			cp_mem_sram_op_req_strobe<=0;
			cp_mem_flash_op_req_strobe<=0;

	
			
			
			//deal with incoming datastream - note it's already been divided into specific-length messages 
			//by the comms controller (which can use OOB signalling like /CS on SPI, or in-band with UART break or message length prefixes)
			if (com_rx_strobe)
			begin
				
`ifdef AUTO_STROBE_BITBANGED_SCLK_SPEEDUP_HACK
				//testing speedup - saves writing both edges of SCLK when bitbanging i2c
				nx4_registers[`OpenNX4_REG_DRIVERR_BITBANG_TEST][`OpenNX4_REG_DRIVERR_BITBANG_BIT_SCLK]<=0;
`endif
				
				//whatever we were doing, if we get the com_rx_start (start of message) signal at the same time as an rx_strobe, 
				//we restart parsing
				if (com_rx_start)begin
					if (com_rx_mode==`COM_MODE_WS2812)begin	//special case for WS2812, we pretend we've been sent an FB write command
						cmd_state<=`CP_STATE_RX_GOT_ADDR_BYTE;	//set up a fake FB write
						rx_id_byte<=(`CP_CMD_MODE_FB<<`CP_CMD_MODE_BIT) | (1<<`CP_CMD_MODE_BIT_WRITE);
						cmd_data_first_byte_flag<=0;
						cp_command_addr<=0;	//write WS2812 pixels to start of fb
						cmd_modulo_counter<=0;
						cp_addr_inc_delay<=0;
					end
					else
					begin	//receive regular first byte of command
						cmd_state<=`CP_STATE_RX_GOT_ID_BYTE;
						rx_id_byte<=com_rx;
					end
				end
				else
				begin: handle_command
					fb_write_mask<=0;
					
					case (cmd_state)
						`CP_STATE_IDLE: begin	//will get bumped out of this by com_rx_start
							end
						`CP_STATE_RX_GOT_ID_BYTE: begin
							if (rx_id_byte[7:4] == self_unit_id  || rx_id_byte[`CP_CMD_MODE_BIT_UNIT_ID+3 -:4]== `UNIT_ID_BROADCAST)begin
								cmd_state<=`CP_STATE_RX_GOT_ADDR_BYTE;
								cmd_data_first_byte_flag<=1;
								cp_command_addr<=com_rx;
								cmd_modulo_counter<=1;
								cp_addr_inc_delay<=0;
							end
							else	begin
								cmd_state<=`CP_STATE_IDLE;	//ignore it if not addressed to us
							end
							end
						//payload comes in here
						`CP_STATE_RX_GOT_ADDR_BYTE: begin
							
							if (cp_addr_inc_delay==0)begin
								//handle incrementing/wrapping the address (which can be register address, sram or framebuffer location, etc)
								cp_command_addr<=cp_command_addr+1;	//post-increment address always
								
								if (nx4_registers[`OpenNX4_REG_CMD_AUTOINC_LEN]!=0)
								begin
									if (cmd_modulo_counter==nx4_registers[`OpenNX4_REG_CMD_AUTOINC_LEN])begin
										//add sign-extended modulo to address
										//note if AUTOINC_LEN==1 then the +1 above gets skipped, so the AUTOINC_MODULO values are slightly non-obvious
										cp_command_addr<=cp_command_addr+ { {8{nx4_registers[`OpenNX4_REG_CMD_AUTOINC_MODULO][7]}},nx4_registers[`OpenNX4_REG_CMD_AUTOINC_MODULO]};
										cmd_modulo_counter<=1;
									end
									else
										cmd_modulo_counter<=cmd_modulo_counter+1;
								end
							end
							else begin
								cp_addr_inc_delay<=cp_addr_inc_delay-1;
							end
							
							
							cmd_data_first_byte_flag<=0;
							
							//// Handle guts of commands received
							case (rx_command_mode)
								`CP_CMD_MODE_GAMMA: begin
									if (is_write)begin	//read not supported
										//write two bytes (msb first) of 12/16 bit 8b->12/16b value, so write 512 bytes total
										if (cp_command_addr[0]==0)begin
											il_msb_latch <= com_rx; //first latch the top few bits 
											cp_addr_inc_delay<=cmd_data_first_byte_flag;
										end
										else 
										begin	//other half of il_din bus is always assigned to com_rx, so on the odd cycle we have the whole word just do the write
											il_wea<=1;
										end
									end
									end
								`CP_CMD_MODE_FB: begin
									if (cmd_data_first_byte_flag)begin
										cp_command_addr<={com_rx,cp_command_addr[7:0]};	//latch MSB of address - (and don't do the +1 increment above)
										cmd_modulo_counter<=1;
										cp_addr_inc_delay<=0; //is_write;
									end
									fb_write_mask<=is_write;	//next com_rx_strobe will active the OE line(s) to the FB

									//may need to do some WS2812b specific munging, e.g. swap RGB order or whatever by manipulating cp_command_addr
									//if (com_rx_mode==`COM_MODE_WS2812)begin	//special case for WS2812, we pretend we've been sent an FB write command

									
									//don't support reading the FB right now (would need to interleave with pixeldriver reads; very doable but.. who cares?)
									//com_tx<=nx4_status[7:0];	//just return status
									//com_tx_strobe<=1;			
											
									end
									
								`CP_CMD_MODE_REGISTERS: begin
									if (is_write)begin
										nx4_registers[cp_command_addr] <= com_rx;
										//return status (i.e. various pin inputs) as a freebie, this allows for some (relatively) fast bit-banged stuff
										com_tx<=nx4_status[7:0];
										com_tx_strobe<=1;	
									end
									else begin	//all registers currently just read back what you wrote to them
										com_tx<=nx4_registers[cp_command_addr];
										com_tx_strobe<=1;	
									end
									end
									
								`CP_CMD_MODE_SRAM: begin
									if (cmd_data_first_byte_flag)begin
										cp_command_addr<={com_rx,cp_command_addr[7:0]};	//latch MSB of address - (and don't do the +1 increment above)
										cp_addr_inc_delay<=is_write;
										cmd_modulo_counter<=1;
										//OpenNX4_REG_UNIT_ID_MEM_ADDR_A16_BIT are bits beyond A15
										cp_mem_sram_op_req_strobe<=~is_write;	//start read now
									end
									else begin
										//todo
										cp_mem_sram_op_req_strobe<=1;	//write strobe to upper layer is only one CLK_40 but data+address remain valid for one byte rx period afterwards
										if (is_write)begin
											//SRAM write
											cp_mem_dout<=com_rx;
										end
										else begin
											com_tx<=cp_mem_din;	//assume read is done by now
											com_tx_strobe<=1;	
										end
									end
									cp_mem_we<=is_write;
									end
								`CP_CMD_MODE_FLASH: begin
									if (cmd_data_first_byte_flag)begin
										cp_command_addr<={com_rx,cp_command_addr[7:0]};	//latch MSB of address - (and don't do the +1 increment above)
										cp_addr_inc_delay<=is_write;
										cmd_modulo_counter<=1;
										cp_mem_flash_op_req_strobe<=~is_write;	//start read now
									end
									else begin
										cp_mem_flash_op_req_strobe<=1;
										if (is_write)begin
											//FLASH write
											cp_mem_dout<=com_rx;
										end
										else begin
											com_tx<=cp_mem_din;	//assume read is done by now
											com_tx_strobe<=1;	
										end
									end
									cp_mem_we<=is_write;
									end
									
								`CP_CMD_MODE_I2C: begin
									//todo i2c interface
									end


								endcase
							end
					endcase

					
				end
			end  //end RX_STROBE handler
			
			else	//not receiving
			begin
`define UART_BOOTUP_MESSAGE
`ifdef UART_BOOTUP_MESSAGE
				//transmit optional bootup message
				if (com_tx_ready)
				begin
					//Send one-time bootup message to say howdy
					if (boot_message_pos>0 && !com_tx_strobe)
					begin: bootmsg
						com_tx<=boot_message[(boot_message_pos*8) -:8]; 
						com_tx_strobe<=1;
						boot_message_pos<=boot_message_pos-1;
					end
				end
			end
`endif


		end
	end


endmodule
