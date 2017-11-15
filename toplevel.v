`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// OpenNX4 - Open source firmware for Barco NX4 tiles
// Company: Bohemian Bits
// Engineer: Richard Aplin (Twitter: @DrTune)
// Copyright (C) 2017 Richard Aplin
// Released under the Attribution-NonCommercial 2.0 Generic (CC BY-NC 2.0)
// You may not use this code or a derived work for commercial purposes unless you have a commercial license, contact drtune@gmail.com
//////////////////////////////////////////////////////////////////////////////////
// 
// Create Date:    13:03:18 10/25/2017 
// Design Name: 
// Module Name:    toplevel 
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

//useful https://www.xilinx.com/support/documentation/sw_manuals/xilinx11/xst.pdf
`include "nx4_header_file.vh"

/* 
Back of envelope timing budget for NX4
CLK_40 40mhz (25ns) - obviously can be PLL'd if required
Each row driver is 3x 16 channel TI drivers (R,G,B) @ 12 bits per channel = 	576 bits per row
IF the dot correction data is loaded also, + 3x16x9 bits = 							432 bits per row

There are 12 row driver outputs
*/

module toplevel(
	 input clock,

	 //pixeldriver pins
    output led_sclk,
    output [6:1] led_l_sin,
    output [6:1] led_r_sin,
    output led_cal_sin,
    input led_xerr,
    output led_mode,
    output led_blank,
    output led_xlat,
    output led_gsclk,
	 
	 //input reset,
	 
	 inout i2c_sda,
	 output i2c_scl,
	 
	 output led_red,
	 output led_yellow,
	 output led_orange,
	 
	 output lt1933_enable,
	 
	 //flash+sram
	 inout mem_s7_15_f7,
    inout mem_s6_14_f6,
    inout mem_s5_13_f5,
    inout mem_s4_12_f4,
    inout mem_s3_11_f3,
    inout mem_s2_10_f2,
    inout mem_s1_9_f1,
    inout mem_s0_8_f0,
    output mem_fa19,
	 output mem_fa18,
	 output mem_fa17,
	 output mem_sa17_fa16,
    output mem_sa16_fa15,
    output mem_sa15_fa14,
    output mem_sa14_fa13,
    output mem_sa13_fa12,
    output mem_sa12_fa11,
    output mem_sa11_fa10,
    output mem_sa10_fa9,
    output mem_sa9_fa8,
    output mem_sa8_fa7,
    output mem_sa7_fa6,
    output mem_sa6_fa5,
    output mem_sa5_fa4,
    output mem_sa4_fa3,
    output mem_sa3_fa2,
    output mem_sa2_fa1,
    output mem_sa1_fa0,
    output mem_sa0_f15,
	 
    output reg mem_we,
	 output sram_ce,
	 output sram_oe,
	 output sram_bhe,
	 output sram_ble,
	 output flash_ce,
    output flash_oe,
    input flash_ry_by,
	 output reg flash_reset,

    input fan_tachometer,

	 //unknown interface to CPLD on the LED driver board
	 inout cpld_p2,
    inout cpld_p3,
    input cpld_p5,
    input cpld_p6,
    inout cpld_p8,
	 inout cpld_p41,
    inout cpld_p42,
    inout cpld_p43,
    inout cpld_p44,
    
    // Input connector from controller or previous NX4 in daisychain
	 input in_conn_p2, 
    input in_conn_p3, 
	 input in_conn_p4,	
    output in_conn_p5,	
    input in_conn_p7,	
    input in_conn_p8,	//must be input
    
	 // Output connecto to next NX4 in daisychain
    input out_conn_p5,
    input out_conn_p8,
    input out_conn_p7,
    input out_conn_p4,
    input out_conn_p3,
    input out_conn_p6


    );

	reg pclock,gclock;
	reg pixel_reset=1;
	wire [15:0] frame_count;
	
	
	wire `NX4_REGISTERS_FLAT;

	integer unpk_idx;
	reg `NX4_REGISTERS_SQUARE;
	always @(*) begin
	`NX4_REGISTERS_FLAT_TO_SQUARE
	end


	wire [7:0] reg_read_bus;
	wire reg_read_strobe;
	
	//todo
	assign lt1933_enable=0;
`define EXT_DATA_BUS mem_s7_15_f7,mem_s6_14_f6,mem_s5_13_f5,mem_s4_12_f4,mem_s3_11_f3,mem_s2_10_f2,mem_s1_9_f1,mem_s0_8_f0
`define 	EXT_ADDRESS_BUS mem_fa19,mem_fa18,mem_fa17,mem_sa17_fa16,mem_sa16_fa15,mem_sa15_fa14,mem_sa14_fa13,mem_sa13_fa12,mem_sa12_fa11,mem_sa11_fa10,mem_sa10_fa9,mem_sa9_fa8,mem_sa8_fa7,mem_sa7_fa6,mem_sa6_fa5,mem_sa5_fa4,mem_sa4_fa3,mem_sa3_fa2,mem_sa2_fa1,mem_sa1_fa0,mem_sa0_f15
	// external sram/flash controller
`ifndef ENABLE_BITBANG_MEMORY	
	reg sram_read_req=1;
	reg flash_read_req=0;
	reg sram_write_req=0;
	reg flash_write_req=0;
	
	reg [17:0] sram_read_address;
	reg [17:0] sram_write_address;
	reg [19:0] flash_read_address;
	reg [19:0] flash_write_address;
	reg [7:0] sram_write_data;
	wire [7:0] sram_read_data;
	wire [7:0] flash_read_data;
	reg [7:0] flash_write_data;
	
	external_memory_controller ext_mem (
    .CLK_40(clock),
	 
	 .ext_address_bus(ext_address_bus),
    .ext_data_bus(ext_data_bus),
    .mem_we(mem_we), 
    .sram_oe(sram_oe), 
    .sram_ce(sram_ce), 
    .sram_bhe(sram_bhe), 
    .sram_ble(sram_ble), 
    .flash_ce(flash_ce), 
    .flash_oe(flash_oe), 
    .flash_ry_by(flash_ry_by), 
    
	 .sram_read_data(sram_read_data), 
    .sram_write_data(sram_write_data), 
    .sram_write_address(sram_write_address), 
    .sram_read_address(sram_read_address), 
    .sram_read_idle(sram_read_idle), 
    .sram_write_req(sram_write_req), 
    .sram_read_req(sram_read_req), 
    .sram_write_idle(sram_write_idle), 
    .flash_read_data(flash_read_data), 
    .flash_read_req(flash_read_req), 
    .flash_write_data(flash_write_data), 
    .flash_write_address(flash_write_address), 
    .flash_read_address(flash_read_address), 
    .flash_read_idle(flash_read_idle), 
    .flash_write_req(flash_write_req), 
    .flash_write_idle(flash_write_idle),
	 .reset(pixel_reset)
	 );
`endif
	
	wire vsync;
	
	//command processor interface to r/w flash and sram
	wire cp_mem_sram_op_req_strobe;
	wire cp_mem_flash_op_req_strobe;
	wire [7:0] cp_mem_dout;
	reg [7:0] cp_mem_din;
	wire cp_mem_we;
	wire [15:0] cp_command_addr;
	 
	wire [`OpenNX4_STATUS_WIDTH-1:0] nx4_status;
	
	wire fb0_wea,fb1_wea;
	wire [`FB_ADDR_WIDTH-1:0] fb_addrb;	//both share same read address bus, are read in parallel
	wire [`FB_ADDR_WIDTH-1:0] fb_addra;	//both share same write address/data bus
	wire [7:0] fb_dina;
	
	wire [7:0] fb0_doutb;
	wire [7:0] fb1_doutb;
	
	//framebuffer (fb) block ram frame buffer 32x36x3 x 8bpp (upconverted to 12 bit when displayed)
	framebuffer fb0 (
		 .wea(fb0_wea), 
		 .addra(fb_addra), 
		 .dina(fb_dina), 
		 .addrb(fb_addrb), 
		 .doutb(fb0_doutb), 
		 .clka(clock), 
		 .clkb(clock),
		 .reset(pixel_reset)
		 );

	framebuffer fb1 (
		 .wea(fb1_wea), 
		 .addra(fb_addra), 
		 .dina(fb_dina), 
		 .addrb(fb_addrb), 
		 .doutb(fb1_doutb), 
		 .clka(clock), 
		 .clkb(clock),
		 .reset(pixel_reset)
		 );

	//ram-based lookup table to convert from 8-bit pixels to 12 bit (due to eye response, 8 bit represents distinguisable tones fine)
	wire il_wea;
	wire [11:0] il_dina;
	wire [7:0] il_addra;
	wire [11:0] il0_doutb;
	wire [7:0] il0_addrb;
	wire [11:0] il1_doutb;
	wire [7:0] il1_addrb;
	wire il_enb=1;

	//we use two lookups (in parallel, one fed by each fb so we can blend them) but written at same time - could use one and time-slice it between fb's but whatever. 
	intensity_lookup_8b_12b intensity_lookup0 (
		 .wea(il_wea),
		 .addra(il_addra), 
		 .dina(il_dina), 
		 .addrb(il0_addrb), 
		 .doutb(il0_doutb), 
		 .clka(clock), 
		 .enb(il_enb),
		 .clkb(clock)
		);

	intensity_lookup_8b_12b intensity_lookup1 (
		 .wea(il_wea),
		 .addra(il_addra), 
		 .dina(il_dina), 
		 .addrb(il1_addrb), 
		 .doutb(il1_doutb), 
		 .clka(clock), 
		 .enb(il_enb),
		 .clkb(clock)
		);

	//sends fb to drivers
	led_array_driver driver (
    .reset(pixel_reset), 
    .led_sclk(led_sclk), 
    .led_l_sin(led_l_sin), 
    .led_r_sin(led_r_sin), 
    .led_cal_sin(led_cal_sin), 
    .led_xerr(led_xerr), 
    .led_mode(led_mode), 
    .led_blank(led_blank), 
    .led_xlat(led_xlat), 
    .led_gsclk(led_gsclk), 
    .pixel_clock(pclock), 
	 .grayscale_clock(gclock),
	 
    .cpld_p8(cpld_p8),
    .cpld_p5(cpld_p5),
	 .cpld_p6(cpld_p6),
	 .cpld_p44(cpld_p44),
	 .cpld_p43(cpld_p43),
	 .cpld_p42(cpld_p42),
	 .cpld_p41(cpld_p41),
	 
	 .frame_count(frame_count),
	 .vsync(vsync),

	 .nx4_registers_flat(nx4_registers_flat),
	 
	 //frame buffers
	 .fb_addr(fb_addrb), 
    .fb0_dout(fb0_doutb),
	 .fb1_dout(fb1_doutb),
	 
	 //8b->12b intensity conversion lookups
	 .il0_addr(il0_addrb), 
	 .il0_dout(il0_doutb),
	 .il1_addr(il1_addrb), 
	 .il1_dout(il1_doutb)

    );

	 wire com_rx_strobe;
	 wire com_rx_start;	//high for first CLK40 of a new command via whatever transport
	 wire com_rx_end_strobe;	//in some transports like UART we don't really know when it's finished unless we implement timeouts but others like SPI and I2C you have transactions
	 wire [7:0] com_rx;
	 wire [7:0] com_tx;
	 wire com_tx_strobe;
	 wire com_tx_ready;
	 wire com_reset_cmd_state;
	 
	 
	//receives data in various protocols & handles writes to the fb and internal state registers like intensity 
	nx4comms comms (
	 .serial_clk_in(in_conn_p8),  //SPI SCK /I2S PCM_BCLK, I2C SCK (no clock stretching supported/reqd)
    .serial_output(in_conn_p5),	//UART host rx, SPI (MISO), I2S (PCMx_DIN). I2C SDA pulldown if active
    .serial_input(in_conn_p4),	//WS2812, UART host tx, SPI (MOSI) and I2S (PCMx_DOUT) all use this. I2C SDA if anyone wants to do it
    .serial_input2(in_conn_p6),	//for 2-bit parallel versions of SPI or I2S
    .serial_select(in_conn_p7),	//if you want SPI with CS, here's where you'd do it

	/*
    .dout_ch1_n(dout_ch1_n), 
    .dout_ch1_p(dout_ch1_p), 
    .dout_ch2_n(dout_ch2_n), 
    .dout_ch2_p(dout_ch2_p), 
    .dout_ch3_n(dout_ch3_n), 
    .dout_ch3_p(dout_ch3_p), 
    */
	 
	 .com_rx_strobe(com_rx_strobe),	//high for one CLK40 cycle when new data is ready, get it while it's hot!
	 .com_rx_start(com_rx_start),	//high for same cycle as above if this is the first byte of a new message (which we know for SPI and I2C for example, not UART)
	 .com_rx_end_strobe(com_rx_end_strobe),	//high for one cycle (not same as RX_STROBE) when we 'know' the message is complete. In some cases we don't know (UART) unless we do a timeout or BREAK or something else
	 .com_reset_cmd_state(com_reset_cmd_state),
    .com_rx_data(com_rx), 
    .com_tx_data(com_tx), 
    .com_tx_ready(com_tx_ready), 
    .com_tx_strobe(com_tx_strobe),

	 .nx4_registers_flat(nx4_registers_flat),
	 
	 .comms_status_led(led_orange),
    .reset(pixel_reset), 
	 
	 .CLK_40(clock)
    );

	
	
	
	
	//this takes data from the comms controller and does stuff with it;
	//interprets command bytes
	//writes to frame buffer
	//set other parameters etc
	command_processor command_processor (
    .il_wea(il_wea), 	//intensity lookup (8b->12b)
    .il_din(il_dina), 
    .il_addr(il_addra), 
    
	 .fb_addr(fb_addra), 	//two framebuffers can write (same addr/data) to either or both 
    .fb_dout(fb_dina), 
    .fb0_we(fb0_wea), 
    .fb1_we(fb1_wea), 
    
    
	 .vsync(vsync), 
	 .frame_count(frame_count), 
	 .status_led_red(led_red),
	 	 
	 .com_rx_strobe(com_rx_strobe),	//high for one CLK40 cycle when new data is ready, get it while it's hot!
	 .com_rx_start(com_rx_start),	//high for same cycle as above if this is the first byte of a new message (which we know for SPI and I2C for example, not UART)
	 .com_rx_end_strobe(com_rx_end_strobe),	//high for one cycle (not same as RX_STROBE) when we 'know' the message is complete. In some cases we don't know (UART) unless we do a timeout or BREAK or something else
    .com_rx(com_rx), 
    .com_tx(com_tx), 
    .com_tx_ready(com_tx_ready), 
    .com_tx_strobe(com_tx_strobe),
	 .com_reset_cmd_state(com_reset_cmd_state),
	 
	 .nx4_registers_flat(nx4_registers_flat),
	 
	 //sram/flash interface
	 .cp_mem_sram_op_req_strobe(cp_mem_sram_op_req_strobe), 
    .cp_mem_flash_op_req_strobe(cp_mem_flash_op_req_strobe), 
    .cp_mem_dout(cp_mem_dout), 
    .cp_mem_din(cp_mem_din), 
    .cp_mem_we(cp_mem_we),
	 .cp_command_addr(cp_command_addr),	//any more bits beyond this (for addressing flash etc) are wired into other registers (paged)
	
	 .reg_read_bus(reg_read_bus),
	 .reg_read_strobe(reg_read_strobe),
	 
	 .nx4_status(nx4_status),
	 
	 .CLK_40(clock),
    .reset(pixel_reset)
    );

	/// -- wiring cmd processor to flash/ram in simple bitbang-style (i.e. no memory controller)
`ifdef ENABLE_BITBANG_MEMORY	
	reg [2:0] cp_mem_cycle_timer;
	wire drive_data_bus=(cp_mem_cycle_timer!=0) & cp_mem_we;
	reg sram_not_flash=0;
	assign {{`EXT_DATA_BUS}} = drive_data_bus ? cp_mem_dout: 8'bz ;
	//address bus is shifted down one as both sram and flash are 16 bit (operating in 8 bit mode)
	assign {{`EXT_ADDRESS_BUS}} = !sram_not_flash ? ({nx4_registers[`OpenNX4_REG_MEM_UPPER_ADDR][4:0],cp_command_addr}):({nx4_registers[`OpenNX4_REG_MEM_UPPER_ADDR][4:0],cp_command_addr[15:1]});
	assign flash_ce=~((cp_mem_cycle_timer!=0)&!sram_not_flash);
	assign flash_oe=~(!sram_not_flash & !cp_mem_we);
	//assign cp_mem_din={{`EXT_DATA_BUS}};
	assign sram_ce=~((cp_mem_cycle_timer!=0)&sram_not_flash);
	assign sram_oe=~(sram_not_flash & !cp_mem_we);
	assign sram_bhe=~cp_command_addr[0];	//lsb of address
	assign sram_ble=~sram_bhe;
	parameter FLASH_CYCLE_TIME=7;
	//asssign mem_we=~(cp_mem_we && (cp_mem_cycle_timer>=1 && cp_mem_cycle_timer<FLASH_CYCLE_TIME));
	
	always @(posedge clock)
	begin	//not using sram/flash memory controller which would arbitrate so we just hook it straight up pretty much
		if (cp_mem_flash_op_req_strobe)begin
			sram_not_flash<=0;
			cp_mem_cycle_timer<=FLASH_CYCLE_TIME;	//stretch flash clock to 7 CLK_40's
		end
		else
		begin
			if (cp_mem_sram_op_req_strobe)begin
				sram_not_flash<=1;
				
				cp_mem_cycle_timer<=1;
				mem_we<=~cp_mem_we;
			end
		end
		
		if (cp_mem_cycle_timer)begin
			if (cp_mem_cycle_timer==2)begin
				mem_we<=~0;	//if writing end write cycle (only applies to flash)
			end
			else
			begin
				if (cp_mem_cycle_timer==1)begin	//sram starts at cycle 1
					if (~cp_mem_we)begin
						cp_mem_din<={`EXT_DATA_BUS};	//sample data
					end
					else
						mem_we<=~0;	//if writing end write cycle (applies to sram), flash ended on previous cycle
				end
				else begin
					mem_we<=~cp_mem_we;
			end
			end
			cp_mem_cycle_timer<=cp_mem_cycle_timer-1;
		end
	end
   
`endif
	/// end wiring cmd processor to flash/ram
	
	// Set up the various things you can read back that are returned as status info
	assign nx4_status={
		cpld_p44,
		cpld_p43,
		cpld_p42,
		cpld_p41,
		cpld_p8,
		cpld_p6,
		cpld_p5,
		cpld_p3,
		cpld_p2,
		flash_ry_by,
		led_xerr,
		i2c_sda
		};
	
	//wire up internal i2c to bitbangable interface as open-drain
	assign i2c_sda=nx4_registers[`OpenNX4_REG_IOCTL][`OpenNX4_REG_IOCTL_BIT_I2C_SDA] ? 1'bz : 0;
	assign i2c_scl=nx4_registers[`OpenNX4_REG_IOCTL][`OpenNX4_REG_IOCTL_BIT_I2C_SCL] ? 1'bz : 0;
	
	
	// blink status led
	reg [32:0] blink_count=0;
	
	assign led_yellow = frame_count[5];
	//assign led_red = blink_count[23];
	
	//reg i2c_sda_reg;
	//assign sda=i2c_sda_reg;
	
	always @(posedge clock)   //board clock input is 40mhz xtal
	begin
		//top level "Main loop"...
		//i2c_sda_reg<=nx4_registers[`OpenNX4_REG_IOCTL][`OpenNX4_REG_IOCTL_BIT_I2C_SDA] ? 1'bz : 0;
		//i2c_scl<=nx4_registers[`OpenNX4_REG_IOCTL][`OpenNX4_REG_IOCTL_BIT_I2C_SCL] ? 1'bz : 0;

	
		begin
			blink_count <= blink_count+1;
			if (blink_count[3] && pixel_reset == 1)	// after 1<<3 clocks, deassert reset
				pixel_reset <= 0;
				flash_reset<=1;
		end

		//test sram
		/*
		if (sram_read_idle)
		begin
			if (blink_count[3])
			begin
				if (sram_read_req==0)
				begin
					//idle - start read
					sram_read_req<=1;
				end
				else
				begin
					//data ready, do whatever
					sram_read_address<=sram_read_address+1;
					//sram_read_req<=0;
				end
			end
			else
			begin	//sit this one out
					sram_read_req<=0;
			end
		end
		*/
		//test sram write
		/*
		if (sram_write_idle)
		begin
			if (sram_write_req==0)
			begin
				//idle - start read
				sram_write_data<=blink_count[7:0];
				sram_write_req<=1;
			end
			else
			begin
				//write complete
				sram_write_address<=sram_write_address+1;
				sram_write_req<=0;
			end
		end
		*/
		/*
		if (flash_write_idle)
		begin
			if (flash_write_req==0)
			begin
				//idle - start read
				flash_write_data<=blink_count[7:0];
				flash_write_req<=1;
			end
			else
			begin
				//write complete
				flash_write_address<=flash_write_address+1;
				flash_write_req<=0;
			end
		end
		*/
		/*
		//test flash read
		if (flash_read_idle)
		begin
			if (flash_read_req==0)
			begin
				//idle - start read
				flash_read_req<=1;
			end
			else
			begin
				//data ready, do whatever
				flash_read_address<=flash_read_address+1;
				//flash_read_req<=0;
			end
		end
		*/

		
		//status_led2 <= blink_count[21];
		pclock <= blink_count[ nx4_registers[`OpenNX4_REG_DRIVECTL][(`OpenNX4_REG_DRIVECTL_BIT_PCLK_DIV0+`OpenNX4_REG_DRIVECTL_BIT_PCLK_BITS-1) -:`OpenNX4_REG_DRIVECTL_BIT_PCLK_BITS ] ];	// pixel clock is /(1<<2) =  40mhz/2=20mhz

		gclock <= blink_count[ nx4_registers[`OpenNX4_REG_DRIVECTL][(`OpenNX4_REG_DRIVECTL_BIT_BCLK_DIV0+`OpenNX4_REG_DRIVECTL_BIT_BCLK_BITS-1) -:`OpenNX4_REG_DRIVECTL_BIT_BCLK_BITS ]]; //greyscale clock is /(1<<0) = 20mhz; it's a 12-bit PWM so the pixel modulation rate is 20mhz/4096=4.8khz which looks good and smooth..  5Mhz looks a bit dotty as you'd imagine
		
	end
	
	
endmodule
