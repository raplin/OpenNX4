`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 	Bohemian Bits 
// Engineer:	Richard Aplin
//
// Create Date:   22:20:40 11/02/2017
// Design Name:   toplevel
// Module Name:   C:/rda/Barco/xilinx/NX4Driver/nx4driver_test.v
// Project Name:  NX4Driver
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: toplevel
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module nx4driver_test;
	// Inputs
	reg clock;
	reg led_xerr;
	reg flash_ry_by;
	reg fan_tachometer;
	reg cpld_p5;
	reg cpld_p6;
	reg in_conn_p2;
	reg in_conn_p3;
	reg tx;	//in_conn_p4
	reg in_conn_p5;
	reg in_conn_p7;
	reg rx; //in_conn_p8;
	reg out_conn_p5;
	reg out_conn_p8;
	reg out_conn_p7;
	reg out_conn_p4;
	reg out_conn_p3;
	reg out_conn_p6;

	// Outputs
	wire led_sclk;
	wire [6:1] led_l_sin;
	wire [6:1] led_r_sin;
	wire led_cal_sin;
	wire led_mode;
	wire led_blank;
	wire led_xlat;
	wire led_gsclk;
	wire i2c_scl;
	wire led_red;
	wire led_yellow;
	wire led_orange;
	wire lt1933_enable;
	wire mem_sa17_fa16;
	wire mem_sa16_fa15;
	wire mem_sa15_fa14;
	wire mem_sa14_fa13;
	wire mem_sa13_fa12;
	wire mem_sa12_fa11;
	wire mem_sa11_fa10;
	wire mem_sa10_fa9;
	wire mem_sa9_fa8;
	wire mem_sa8_fa7;
	wire mem_sa7_fa6;
	wire mem_sa6_fa5;
	wire mem_sa5_fa4;
	wire mem_sa4_fa3;
	wire mem_sa3_fa2;
	wire mem_sa2_fa1;
	wire mem_sa1_fa0;
	wire mem_sa0_f15;
	wire mem_we;
	wire sram_oe;
	wire sram_bhe;
	wire sram_ble;
	wire flash_ce;
	wire flash_oe;
	wire in_conn_p4;

	// Bidirs
	wire i2c_sda;
	wire mem_s7_15_f7;
	wire mem_s6_14_f6;
	wire mem_s5_13_f5;
	wire mem_s4_12_f4;
	wire mem_s3_11_f3;
	wire mem_s2_10_f2;
	wire mem_s1_9_f1;
	wire mem_s0_8_f0;
	wire cpld_p2;
	wire cpld_p3;
	wire cpld_p8;
	wire cpld_p41;
	wire cpld_p42;
	wire cpld_p43;
	wire cpld_p44;

	// Instantiate the Unit Under Test (UUT)
	toplevel uut (
		.clock(clock), 
		.led_sclk(led_sclk), 
		.led_l_sin(led_l_sin), 
		.led_r_sin(led_r_sin), 
		.led_cal_sin(led_cal_sin), 
		.led_xerr(led_xerr), 
		.led_mode(led_mode), 
		.led_blank(led_blank), 
		.led_xlat(led_xlat), 
		.led_gsclk(led_gsclk), 
		.i2c_sda(i2c_sda), 
		.i2c_scl(i2c_scl), 
		.led_red(led_red), 
		.led_yellow(led_yellow), 
		.led_orange(led_orange), 
		.lt1933_enable(lt1933_enable), 
		.mem_s7_15_f7(mem_s7_15_f7), 
		.mem_s6_14_f6(mem_s6_14_f6), 
		.mem_s5_13_f5(mem_s5_13_f5), 
		.mem_s4_12_f4(mem_s4_12_f4), 
		.mem_s3_11_f3(mem_s3_11_f3), 
		.mem_s2_10_f2(mem_s2_10_f2), 
		.mem_s1_9_f1(mem_s1_9_f1), 
		.mem_s0_8_f0(mem_s0_8_f0), 
		.mem_sa17_fa16(mem_sa17_fa16), 
		.mem_sa16_fa15(mem_sa16_fa15), 
		.mem_sa15_fa14(mem_sa15_fa14), 
		.mem_sa14_fa13(mem_sa14_fa13), 
		.mem_sa13_fa12(mem_sa13_fa12), 
		.mem_sa12_fa11(mem_sa12_fa11), 
		.mem_sa11_fa10(mem_sa11_fa10), 
		.mem_sa10_fa9(mem_sa10_fa9), 
		.mem_sa9_fa8(mem_sa9_fa8), 
		.mem_sa8_fa7(mem_sa8_fa7), 
		.mem_sa7_fa6(mem_sa7_fa6), 
		.mem_sa6_fa5(mem_sa6_fa5), 
		.mem_sa5_fa4(mem_sa5_fa4), 
		.mem_sa4_fa3(mem_sa4_fa3), 
		.mem_sa3_fa2(mem_sa3_fa2), 
		.mem_sa2_fa1(mem_sa2_fa1), 
		.mem_sa1_fa0(mem_sa1_fa0), 
		.mem_sa0_f15(mem_sa0_f15), 
		.mem_we(mem_we), 
		.sram_oe(sram_oe), 
		.sram_bhe(sram_bhe), 
		.sram_ble(sram_ble), 
		.flash_ce(flash_ce), 
		.flash_oe(flash_oe), 
		.flash_ry_by(flash_ry_by), 
		.fan_tachometer(fan_tachometer), 
		.cpld_p2(cpld_p2), 
		.cpld_p3(cpld_p3), 
		.cpld_p5(cpld_p5), 
		.cpld_p6(cpld_p6), 
		.cpld_p8(cpld_p8), 
		.cpld_p41(cpld_p41), 
		.cpld_p42(cpld_p42), 
		.cpld_p43(cpld_p43), 
		.cpld_p44(cpld_p44), 
		.in_conn_p2(in_conn_p2), 
		.in_conn_p3(in_conn_p3), 
		.in_conn_p4(rx), 
		.in_conn_p5(tx), 
		.in_conn_p7(in_conn_p7), 
		.in_conn_p8(in_conn_p8), 
		.out_conn_p5(out_conn_p5), 
		.out_conn_p8(out_conn_p8), 
		.out_conn_p7(out_conn_p7), 
		.out_conn_p4(out_conn_p4), 
		.out_conn_p3(out_conn_p3), 
		.out_conn_p6(out_conn_p6)
	);

	initial begin
		// Initialize Inputs
		clock = 0;
		led_xerr = 0;
		flash_ry_by = 0;
		fan_tachometer = 0;
		cpld_p5 = 0;
		cpld_p6 = 0;
		in_conn_p2 = 0;
		in_conn_p3 = 0;
		tx=0;
		in_conn_p5 = 0;
		in_conn_p7 = 0;
		//rx in_conn_p8 = 0;
		out_conn_p5 = 0;
		out_conn_p8 = 0;
		out_conn_p7 = 0;
		out_conn_p4 = 0;
		out_conn_p3 = 0;
		out_conn_p6 = 0;

		#100;

	end
      

	always begin
		// 40mhz clock
		forever #12.5 clock=~clock;
	end

// Generate this file from "NX4Comms.py" with the "test" option
// e.g.
// python NX4Comms.py test
// which outputs verilog to drive the simulated chip
`include "test_scripts_gen\uart_openNX4_toplevel_test_autogen.v"
		
endmodule

