//////////////////////////////////////////////////////////////////////////////////
// OpenNX4 - Open source firmware for Barco NX4 tiles
// Company: Bohemian Bits
// Engineer: Richard Aplin (Twitter: @DrTune)
// Copyright (C) 2017 Richard Aplin
// Released under the Attribution-NonCommercial 2.0 Generic (CC BY-NC 2.0)
// You may not use this code or a derived work for commercial purposes unless you have a commercial license, contact drtune@gmail.com
//////////////////////////////////////////////////////////////////////////////////


/// --- IMPORTANT DEFAULTS ///
`define DEFAULT_PIXEL_CLOCK_DIVIDER 2  //20mhz/(1<<n) , so 2=1<<2= 20/4=5mhz, which is pretty good
`define DEFAULT_GSPWM_CLOCK_DIVIDER 0	//20/(1<<n) = 10mhz gsclock


`define INTERNAL_PIXEL_WIDTH_BITS 12	//12 or 16 bits per RGB channel. 16 uses temporal dithering, is not fully implemented yet
//only enable this for >12 bit internal formats
//`define TEMPORAL_DITHER

//framebuffer is currently 8 bit

//UART speed...
`define DEFAULT_BAUD_MULTIPLIER 3	// 115200 << n (in this case = 465k)
//Note with a 40mhz clock (and due to the way it's currently implemented) the actual baud rates will be;
/*
0 115273
1 231213
2 465116
3 930232	  <<ftdi supports 923077 which is close enough
4 1904761
5 4000000
6 8000000  <<supported by highspeed FTDI chips, e.g. FT232H, 2232H
7 20000000  <<good luck with this

It'd be possible to more closely match FT232H baud rates at high speeds (although 8MBaud is spot on), see 
http://www.ftdichip.com/Support/Documents/AppNotes/AN_120_Aliasing_VCP_Baud_Rates.pdf
*/

/// --- DEBUG STUFF
`define ENABLE_BITBANG_MEMORY 1
//`define BITBANG_LED_DRIVE_TEST_MODE 1
`define EXT_ADDRESS_BUS_WIDTH 21


//note in nx4comms.v some of these values are hardwired; todo tidy
`define COM_MODE_SPI				0	//SPI (or I2S), usually CS is not required, so just MOSI, MISO, CLK
`define COM_MODE_SPI_2BIT		1	//two bits at a time mode (requires host to run synchronous 2-bit SPI or I2S; both MOSI bits must be in lock-step)
`define COM_MODE_WS2812			2	//pretends to be a string of WS2812's :-)
`define COM_MODE_UART			3	//UART will go as fast as you can drive it :-)
`define COM_MODE_I2C				4	//todo if anyone wants it; a UART is basically easier if you're talking to an MCU but you might want I2C for specific reasons
//..smoke signals, telepathy etc
`define COM_MODE_BARCO			6  //future todo(?) (likely output only) for driving downstream NX4s with original Barco FPGA firmware
`define COM_MODE_DISABLED		7  //used to disable output when enumerating devices

`define DEFAULT_IN_COM_MODE	`COM_MODE_UART
`define DEFAULT_OUT_COM_MODE	`COM_MODE_UART
`define BASELINE_UART_BAUD 115200	//this can be multiplied by 2x, 4x etc with a register setting. 

///////////////////////////////////////

////////////////////////////////////////////

/*
//First byte of data stream defines which unit id is recipient, and the command mode,
//which can be addressing SRAM, flash, frame buffer, registers, internal i2c bus, etc
//sram, flash, registers, frame buffer,cmd,i2c,gamma correction table,0
*/
`define CP_CMD_MODE_REGISTERS 0
`define CP_CMD_MODE_FB			1
`define CP_CMD_MODE_SRAM		2
`define CP_CMD_MODE_FLASH		3
`define CP_CMD_MODE_I2C			4	//todo do stuff on internal i2c bus
`define CP_CMD_MODE_GAMMA		5	//this could be moved to appear like a psuedo-register instead if you want to reuse this slot, it's not very important
//...
`define CP_CMD_MODE_CMD			7	//todo: trigger execution of some onboard function todo
`define CP_CMD_MODE_BIT			0	//bits 0..2 

`define CP_CMD_MODE_BIT_WRITE 3
`define CP_CMD_MODE_BIT_UNIT_ID 4 //..4 bit unit ID

// 
`define CP_STATE_IDLE 				0
`define CP_STATE_RX_GOT_ID_BYTE 	1
`define CP_STATE_RX_GOT_ADDR_BYTE 2

// Send messages on this ID to go to all units
`define UNIT_ID_BROADCAST 			15

//------------------- Register list -------------------
`define OpenNX4_REG_SYSCTL				0
//  Bit defs for SYSCTL reg
`define  OpenNX4_REG_SYSCTL_BIT_FB0_WRITE 	0	//if 1, writes to the Framebuffer will write to FB0
`define  OpenNX4_REG_SYSCTL_BIT_FB1_WRITE 	1	//if 1, writes to the Framebuffer will write to FB1 - it's perfectly legal to write to both (possibly useful) at the same time. 
`define  OpenNX4_REG_SYSCTL_BIT_MSG_LENGTH_PREFIX0 	2	//size of length prefix on messages received over SPI or UART; 0..2 (default 1)
`define  OpenNX4_REG_SYSCTL_BIT_MSG_LENGTH_PREFIX1 	3	//
`define  OpenNX4_REG_SYSCTL_BIT_AUTO_BUFFER_FLIP		4	//transition to other framebuffer automatically after each FB write operation completes
`define  OpenNX4_REG_SYSCTL_BIT_UPDATE_DC		5	//if set, writes a frame of dot correct values from OpenNX4_REG_DOT_CORRECT_TEST (same value to every pixel) - turn off for normal display output. Dotcorrect is loaded automatically one time at startup
//--
`define OpenNX4_REG_IOCTL				1
`define  OpenNX4_REG_IOCTL_BIT_RED_LED			0	//1=lit
`define  OpenNX4_REG_IOCTL_BIT_YELLOW_LED		1	//1=lit
`define  OpenNX4_REG_IOCTL_BIT_AMBER_LED		2	//1=lit
`define  OpenNX4_REG_IOCTL_BIT_I2C_SCL			3	//1=floating,0=pulled low
`define  OpenNX4_REG_IOCTL_BIT_I2C_SDA			4	//1=floating,0=pulled low
`define  OpenNX4_REG_IOCTL_BIT_UART_BAUD_0		5	//000=115200, 001=115k*2, 010=115k*4, 011=115k*8 ...  111=115k*128=14mbit
`define  OpenNX4_REG_IOCTL_BIT_UART_BAUD_BITCOUNT 3
//--
`define OpenNX4_REG_COMCTL				2
`define  OpenNX4_REG_COMCTL_BIT_IN_COM_MODE 	0	//3 bits - input format (UART etc)
`define  OpenNX4_REG_COMCTL_BIT_OUT_COM_MODE 3  //format for output to OUT connector (i.e. daisychain) - set this to COM_MODE_DISABLED to prevent data passthru during initial enumeration
//--
`define OpenNX4_REG_UNIT_ID			3	//4 bit
//-- Modulo lets you write a rectangular area to frame buffer (every N pixels, add M to skip to start of next rectangle row)
// or can be used for efficient bit-banging where you set it to look over one or more of the bitbang registers
`define OpenNX4_REG_CMD_AUTOINC_LEN		4	//after N command bytes, add the modulo to the address
`define OpenNX4_REG_CMD_AUTOINC_MODULO	5	//signed 8 bit modulo to be added to address after N command bytes
//--
`define OpenNX4_REG_MEM_UPPER_ADDR  6 //bits 0..4 are Address 16-20 for flash/sram accesses

`define OpenNX4_REG_FB0_INTENSITY 	7	//can display one or other of frame buffers or blend them  (0=off, 0xff=full) 
`define OpenNX4_REG_FB1_INTENSITY 	8
// --
`define OpenNX4_REG_DRIVECTL		 	9	//controls pixel and greyscale PWM clock dividers 40mhz/(1<<n)
`define  OpenNX4_REG_DRIVECTL_BIT_PCLK_DIV0  0  //..bits 0..3
`define  OpenNX4_REG_DRIVECTL_BIT_PCLK_BITS  4  
`define  OpenNX4_REG_DRIVECTL_BIT_BCLK_DIV0  4  //..bits 4..7
`define  OpenNX4_REG_DRIVECTL_BIT_BCLK_BITS  4  
//--
//`define OpenNX4_REG_FB_BLEND			9
//--
//`define OpenNX4_REG_FB_BLEND_TARGET	10
//-
//`define OpenNX4_REG_FB_BLEND_STEP	11
//...
`define OpenNX4_REG_DOT_CORRECT_TEST 11	//value written to every pixel as dot-correction

//remaining are test registers for hax0ring
`define OpenNX4_REG_CPLD_BITBANG_TEST 		12
//for poking at the cpld - when we figure out row scanning etc we can get rid of this
`define  OpenNX4_REG_CPLD_BITBANG_BIT_PIN2 0 		
`define  OpenNX4_REG_CPLD_BITBANG_BIT_PIN3 1 		
`define  OpenNX4_REG_CPLD_BITBANG_BIT_PIN8 2 		
`define  OpenNX4_REG_CPLD_BITBANG_BIT_PIN41 3 		
`define  OpenNX4_REG_CPLD_BITBANG_BIT_PIN42 4 		
`define  OpenNX4_REG_CPLD_BITBANG_BIT_PIN43 5 		
`define  OpenNX4_REG_CPLD_BITBANG_BIT_PIN44 6 		

// Test stuff; obsolete, reuse this if you like
`define OpenNX4_REG_DRIVERL_BITBANG_TEST	13		
//	bit 0..5 are pixel driver outputs
`define  OpenNX4_REG_DRIVERL_BITBANG_BIT_CAL_SIN	6
`define  OpenNX4_REG_DRIVERL_BITBANG_BIT_MODE		7

`define OpenNX4_REG_DRIVERR_BITBANG_TEST	14	
//	bit 0..5 are pixel driver outputs
`define  OpenNX4_REG_DRIVERR_BITBANG_BIT_XLAT		6
`define  OpenNX4_REG_DRIVERR_BITBANG_BIT_SCLK		7

`define OpenNX4_REG_TEST_PIXEL	15

//... we support 16 registers
`define OpenNX4_REG_COUNT 16

// --------------------------------------------------------------------------------------------------------

//whatever we want in the status byte(s) you can read back
`define OpenNX4_STATUS_WIDTH 8
`define OpenNX4_STATUS0_BIT_I2C_SDA   0

// --------------------------------------------------------------------------------------------------------

`define SPI_USES_CS 0

// --------------------------------------------------------------------------------------------------------

//WTF is going on with (regular) verilog not letting you pass 2d arrays? 
//thanks to http://www.edaboard.com/thread80929.html for sanity saving
`define PACK_ARRAY(PK_WIDTH,PK_LEN,PK_SRC,PK_DEST)    for (pk_idx=0; pk_idx<(PK_LEN); pk_idx=pk_idx+1) assign PK_DEST[((PK_WIDTH)*pk_idx+((PK_WIDTH)-1)) -:PK_WIDTH] = PK_SRC[pk_idx][((PK_WIDTH)-1):0];
`define UNPACK_ARRAY(PK_WIDTH,PK_LEN,PK_DEST,PK_SRC) for (unpk_idx=0; unpk_idx<(PK_LEN); unpk_idx=unpk_idx+1) PK_DEST[unpk_idx][((PK_WIDTH)-1):0] = PK_SRC[((PK_WIDTH)*unpk_idx+(PK_WIDTH-1)) -:PK_WIDTH ]; 

`define NX4_REGISTERS_FLAT 	[(8*`OpenNX4_REG_COUNT)-1:0] nx4_registers_flat
`define NX4_REGISTERS_SQUARE 	[7:0] nx4_registers[(`OpenNX4_REG_COUNT)-1:0]
`define NX4_REGISTERS_FLAT_TO_SQUARE 	`UNPACK_ARRAY(8,`OpenNX4_REG_COUNT,nx4_registers,nx4_registers_flat)
`define NX4_REGISTERS_SQUARE_TO_FLAT	`PACK_ARRAY(8,`OpenNX4_REG_COUNT,nx4_registers_flat,nx4_registers)

// --------------------------------------------------------------------------------------------------------


`define FB_ADDR_WIDTH 12  //12 bit bus b/c 32*36*3= 3456 bytes 
