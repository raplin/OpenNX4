`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// OpenNX4 - Open source firmware for Barco NX4 tiles
// Company: Bohemian Bits
// Engineer: Richard Aplin (Twitter: @DrTune)
// Copyright (C) 2017 Richard Aplin
// Released under the Attribution-NonCommercial 2.0 Generic (CC BY-NC 2.0)
// You may not use this code or a derived work for commercial purposes unless you have a commercial license, contact drtune@gmail.com
//////////////////////////////////////////////////////////////////////////////////

//     Note this file is incomplete at present and not used

// Create Date:    23:22:20 10/29/2017 
// Design Name: 
// Module Name:    external_memory_controller 
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

// SRAM is http://www.cypress.com/file/42801/download 
// 256x16, 10ns.  Currently we just use a 40mhz clock but we could use both edges (or fire up a PLL)
// In this case it's wired with an 8 bit data bus

// Flash is http://www.cypress.com/file/217501/download
// 2mbit/8 'access time as fast as 70ns'
//  require unlock sequence to write
// max address to output delay is 70ns, CE delay is also 70ns, OE is 30ns 
module external_memory_controller(
	//external device IOs
	output reg [19:0] ext_address_bus,
	inout [7:0] ext_data_bus,
	output reg mem_we,	//active low
	output reg sram_oe,	//active low
	output reg sram_bhe, //active low
	output reg sram_ble, //active low
	output reg flash_ce, //active low
   output reg flash_oe, //active low
   input flash_ry_by,

	input CLK_40,

	//internal interface
	
	output reg [7:0] sram_read_data,
	input [7:0] sram_write_data,
	input [17:0] sram_write_address,
	input [17:0] sram_read_address,
	output reg sram_read_idle,
	input sram_read_req,
	input sram_write_req,
	output reg sram_write_idle,
	
	output reg [7:0] flash_read_data,
	input flash_read_req,
	input [7:0] flash_write_data,
	input [19:0] flash_write_address,
	input [19:0] flash_read_address,
	output reg flash_read_idle,
	input flash_write_req,
	output reg flash_write_idle,
	
	input reset
   );

//this is clocked on both edges of the 40mhz (25ns) clock for the sram (10ns) but the flash is 70ns or so
parameter STATE_IDLE 				=0;
parameter STATE_SRAM_READ			=1;
parameter STATE_SRAM_WRITE			=2;
parameter STATE_FLASH_READ_SETUP	=3;
parameter STATE_FLASH_READ			=4;
parameter STATE_FLASH_WRITE_SETUP=5;
parameter STATE_FLASH_WRITE		=6;

parameter FLASH_ACCESS_TIME_CYCLES=3;  //3 25ns cycles = 75ns

	reg [3:0]state;
	
	reg ext_data_is_output=0;
	reg [7:0] ext_data_out;
	
	reg [2:0] delay_cycles;
	
	assign ext_data_bus = ext_data_is_output ? ext_data_out : 8'bz;
	integer next_state;
	
	always @(posedge CLK_40 or posedge reset)
	begin
		if (reset) begin
			ext_address_bus<=0;
			mem_we<=1;
			sram_bhe<=1;
			sram_ble<=0;
			flash_ce<=1;
			flash_oe<=1;
			flash_read_data<=0;
			flash_read_idle<=1;
			sram_read_data<=0;
			sram_read_idle<=1;
			sram_oe<=1;
			state<=STATE_IDLE;
			flash_write_idle<=1;
			sram_write_idle<=1;
		end
		else
		
		begin
			//client de-asserts read_req when they've consumed the data from the last read
			
			//when a client requests an operation we immediately go non-idle on that; the rising edge of idle will be when the operation has been scheduled and done
			//note that we don't necessarily sample the address (or data if writing) immediately on deasserting idle - it depends if another operation is running
			//we could always latch these if it'd be useful but it's just more gates
			//basically the client should also keep asserting their _req and addr/data until the operation is done although in practice once we're out of STATE_IDLE it's been latched.
			
			if (sram_read_req && sram_read_idle) sram_read_idle<=0;
			if (sram_write_req && sram_write_idle) sram_write_idle<=0;
			if (flash_read_req && flash_read_idle) flash_read_idle<=0;
			if (flash_write_req && flash_write_idle) flash_write_idle<=0;
			
			if (delay_cycles>0)begin
				delay_cycles<=delay_cycles-1;
			end
			else
			begin
				next_state=state;
				if (state!=STATE_IDLE)
				begin
					case (state)
									
					STATE_SRAM_READ:
									begin
										sram_read_data<=ext_data_bus;
										sram_read_idle<=1;
										sram_oe<=1;
										next_state=STATE_IDLE;
									end
					STATE_FLASH_READ:	
									begin
										flash_read_data<=ext_data_bus;
										flash_read_idle<=1;
										flash_oe<=1;
										next_state=STATE_IDLE;
									end
									
					STATE_SRAM_WRITE:
									begin
										mem_we<=1;
										sram_write_idle<=1;
										next_state=STATE_IDLE;
									end

					STATE_FLASH_WRITE:
									begin	//todo this should probably stall until any previous write has finished
										mem_we<=1;
										flash_write_idle<=1;
										next_state=STATE_IDLE;
									end
		
					default:		begin
									end
					endcase		
				end
				
				if (next_state==STATE_IDLE)
				begin
					//first priority is sram reads
					if (sram_read_req) 
					begin
						next_state=STATE_SRAM_READ;
						flash_oe<=1;
						flash_ce<=1;
						sram_oe<=0;
						mem_we<=1;
						sram_ble<=sram_write_address[0];
						sram_bhe<=~sram_write_address[0];
						ext_data_is_output<=0;
						ext_address_bus<=sram_read_address[17:1];	//samples address here
						
					end
					else
						sram_read_idle<=1;
					
					if (next_state==STATE_IDLE)
					begin
						if (sram_write_req)
						begin
							next_state=STATE_SRAM_WRITE;
							flash_oe<=1;
							flash_ce<=1;
							mem_we<=0;
							sram_oe<=1;
							ext_data_out<=sram_write_data;
							ext_data_is_output<=1;
							sram_ble<=sram_write_address[0];
							sram_bhe<=~sram_write_address[0];
							ext_address_bus<=sram_write_address[17:1];
						end
						else
							sram_write_idle<=1;
					end
					
					if (next_state==STATE_IDLE)
					begin
						if (flash_read_req)
						begin
							next_state=STATE_FLASH_READ;
							flash_oe<=0;
							flash_ce<=0;
							sram_oe<=1;
							mem_we<=1;
							ext_data_is_output<=0;
							ext_address_bus<=flash_read_address[19:0];
							delay_cycles<=FLASH_ACCESS_TIME_CYCLES;
						end
						else
							flash_read_idle<=1;
					end
					
					if (next_state==STATE_IDLE)
					begin
						if (flash_write_req)
						begin
							next_state=STATE_FLASH_WRITE;
							flash_oe<=1;
							flash_ce<=0;
							mem_we<=0;
							sram_oe<=1;
							ext_data_out<=flash_write_data;
							ext_data_is_output<=1;
							ext_address_bus<=flash_write_address[19:0];
							delay_cycles<=FLASH_ACCESS_TIME_CYCLES;
						end
						else
							flash_write_idle<=1;
					end
					
					if (next_state==STATE_IDLE)
					begin
						flash_oe<=1;
						flash_ce<=1;
						sram_oe<=1;
						mem_we<=1;
					end
				end
				state<=next_state;
			end
		end
		
	end

endmodule
