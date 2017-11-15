`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   20:45:26 11/01/2017
// Design Name:   uart
// Module Name:   C:/rda/Barco/xilinx/NX4Driver/uart_test.v
// Project Name:  NX4Driver
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: uart
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module uart_test;

	// Inputs
	reg rx;
	reg [15:0] baud_rate;
	reg [7:0] tx_data;
	reg tx_data_strobe;
	reg CLK_40;
	reg reset;

	// Outputs
	wire tx;
	wire [7:0] rx_data;
	wire rx_data_strobe;
	wire tx_busy;

	// Instantiate the Unit Under Test (UUT)
	uart uut (
		.rx(rx), 
		.tx(tx), 
		.rx_data(rx_data), 
		.baud_rate(baud_rate), 
		.rx_data_strobe(rx_data_strobe), 
		.tx_data(tx_data), 
		.tx_data_strobe(tx_data_strobe), 
		.tx_busy(tx_busy), 
		.CLK_40(CLK_40), 
		.reset(reset)
	);

	initial begin
		// Initialize Inputs
		rx = 0;
		baud_rate = 8; //40000000/115200;
		tx_data = 0;
		tx_data_strobe = 0;
		CLK_40 = 0;
		reset = 0;

		// Add stimulus here
		#10
		reset=1;
		#50
		reset=0;
	end
	
	always begin
		forever #12.5 CLK_40=~CLK_40;
	end

	always begin
		forever #12.5 rx<=tx;
	end

	always begin
		#100 tx_data <= 'hc1;  tx_data_strobe<=1;
		#100 tx_data_strobe<=0;
		#1000 tx_data_strobe<=0;
	end

//	always begin
//		#200
//`define SPICLK2 100
//`define mosi		
		//`include "spi_tester_gen.v"
//	end
  
endmodule

