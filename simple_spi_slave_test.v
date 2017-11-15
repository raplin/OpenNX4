`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   21:50:51 10/27/2017
// Design Name:   simple_spi_slave
// Module Name:   C:/rda/Barco/xilinx/NX4Driver/simple_spi_slave_test.v
// Project Name:  NX4Driver
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: simple_spi_slave
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module simple_spi_slave_test;

	integer i;
	
	// Inputs
	reg mosi;
	reg sck;
	reg CLK_40;
	reg reset;
	reg tx_data_strobe;
	reg cs;
	reg [7:0] tx_data;
	
	// Outputs
	wire rx_data_strobe;
	wire miso;
	wire [7:0] rx_data;
	wire rx_ready;

	wire rx_start;
	wire rx_end_strobe;

	// Instantiate the Unit Under Test (UUT)
	simple_spi_slave uut (
		.mosi(mosi), 
		.miso(miso), 
		.sck(sck), 
		.cs(cs), 
		.CLK_40(CLK_40), 
		.rx_data(rx_data), 
		.tx_data(tx_data), 
		.rx_data_strobe(rx_data_strobe), 
		.tx_data_strobe(tx_data_strobe), 
		.reset(reset),
		.rx_start(rx_start),
		.rx_end_strobe(rx_end_strobe)
	);

	
	initial begin
		// Initialize Inputs
		mosi = 0;
		sck = 0;
		CLK_40 = 0;
		tx_data_strobe = 0;
		reset = 0;
		cs=1;
		// Wait 100 ns for global reset to finish
		#10
		reset=1;
		#50
		reset=0;

	end
	
	always begin
		forever #12.5 CLK_40=~CLK_40;
	end
	
	`include "spi_tester_gen.v"
	   
endmodule

