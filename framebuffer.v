`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// OpenNX4 - Open source firmware for Barco NX4 tiles
// Company: Bohemian Bits
// Engineer: Richard Aplin (Twitter: @DrTune)
// Copyright (C) 2017 Richard Aplin
// Released under the Attribution-NonCommercial 2.0 Generic (CC BY-NC 2.0)
// You may not use this code or a derived work for commercial purposes unless you have a commercial license, contact drtune@gmail.com
//////////////////////////////////////////////////////////////////////////////////
// Create Date:    05:55:51 10/27/2017 
// Design Name: 
// Module Name:    framebuffer 
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
module framebuffer(

	input [0 : 0] wea,
	input [11 : 0] addra,
	input [7 : 0] dina,
	input [11 : 0] addrb,
	output [7 : 0] doutb,
	input clka,
	input clkb,
	input reset
	);

	
	blk_mem_gen_v7_3 fb0 (
		 .clka(clka), 
		 .ena(1'b1), 
		 .wea(wea), 
		 .addra(addra), 
		 .dina(dina), 
		 .clkb(clkb), 
		 .rstb(reset), 
		 .addrb(addrb), 
		 .doutb(doutb)
		 );

   
endmodule
