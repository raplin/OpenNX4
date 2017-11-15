`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// OpenNX4 - Open source firmware for Barco NX4 tiles
// Company: Bohemian Bits
// Engineer: Richard Aplin (Twitter: @DrTune)
// Copyright (C) 2017 Richard Aplin
// Released under the Attribution-NonCommercial 2.0 Generic (CC BY-NC 2.0)
// You may not use this code or a derived work for commercial purposes unless you have a commercial license, contact drtune@gmail.com
//////////////////////////////////////////////////////////////////////////////////
// Create Date:    12:55:36 10/25/2017 
// Design Name: 
// Module Name:    led_array_driver 
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

// http://www.ti.com/lit/ds/symlink/tlc5941.pdf
`include "nx4_header_file.vh"


module led_array_driver(
    output reg led_sclk,
    input led_xerr,
`ifdef BITBANG_LED_DRIVE_TEST_MODE
    output led_cal_sin,
    output led_mode,
    output led_xlat,
    output [6:1] led_l_sin,
    output [6:1] led_r_sin,
`else
    output reg [6:1] led_l_sin,
    output reg [6:1] led_r_sin,
    output reg led_cal_sin,
    output reg led_mode,
    output reg led_xlat,
`endif
    output reg led_blank,
    output reg led_gsclk,
	 //unknown pins to cpld on driver board
	 inout cpld_p2,
    inout cpld_p3,
    input cpld_p5,
    input cpld_p6,
    inout cpld_p8,
	 inout cpld_p41,
    inout cpld_p42,
    inout cpld_p43,
    inout cpld_p44,
	  //connects to core
	 input pixel_clock,
	 input grayscale_clock,
	 input reset,
	 output reg [15:0] frame_count,
	 output reg vsync,
	 //frame buffer
	 output reg [`FB_ADDR_WIDTH-1:0] fb_addr,
	 input [7:0] fb0_dout,
	 input [7:0] fb1_dout,
	 input `NX4_REGISTERS_FLAT, 
	
	 input  [11:0] il0_dout,	//intensity lookup; runs at grayscale clock rate because we need to do two lookups per pixel (dual frame buffers)
	 output reg [7:0] il0_addr,
	 input  [11:0] il1_dout,
	 output reg [7:0] il1_addr
    );

	reg [11:0] blanking_clock=0;	
	reg pixel_clock_mask=1;
	
	integer unpk_idx;
	reg `NX4_REGISTERS_SQUARE;
	always @(*) begin
	`NX4_REGISTERS_FLAT_TO_SQUARE
	end

	//assign cpld_p8=pixel_clock;


	assign cpld_p2=nx4_registers[`OpenNX4_REG_CPLD_BITBANG_TEST][`OpenNX4_REG_CPLD_BITBANG_BIT_PIN2];
	assign cpld_p3=nx4_registers[`OpenNX4_REG_CPLD_BITBANG_TEST][`OpenNX4_REG_CPLD_BITBANG_BIT_PIN3];
	assign cpld_p8=nx4_registers[`OpenNX4_REG_CPLD_BITBANG_TEST][`OpenNX4_REG_CPLD_BITBANG_BIT_PIN8];
	assign cpld_p41=nx4_registers[`OpenNX4_REG_CPLD_BITBANG_TEST][`OpenNX4_REG_CPLD_BITBANG_BIT_PIN41];
	assign cpld_p42=nx4_registers[`OpenNX4_REG_CPLD_BITBANG_TEST][`OpenNX4_REG_CPLD_BITBANG_BIT_PIN42];
	assign cpld_p43=nx4_registers[`OpenNX4_REG_CPLD_BITBANG_TEST][`OpenNX4_REG_CPLD_BITBANG_BIT_PIN43];
	assign cpld_p44=nx4_registers[`OpenNX4_REG_CPLD_BITBANG_TEST][`OpenNX4_REG_CPLD_BITBANG_BIT_PIN44];

`ifdef BITBANG_LED_DRIVE_TEST_MODE
	//control the lines with register writes directly for testing
	//haven't implemented a register read bus yet
	
	reg last_sclk=0;
	
	assign led_l_sin=nx4_registers[`OpenNX4_REG_DRIVERL_BITBANG_TEST][5:0];
	assign led_r_sin=nx4_registers[`OpenNX4_REG_DRIVERR_BITBANG_TEST][5:0];
	assign led_cal_sin=nx4_registers[`OpenNX4_REG_DRIVERL_BITBANG_TEST][`OpenNX4_REG_DRIVERL_BITBANG_BIT_CAL_SIN];
	assign led_mode=nx4_registers[`OpenNX4_REG_DRIVERL_BITBANG_TEST][`OpenNX4_REG_DRIVERL_BITBANG_BIT_MODE];
   assign led_xlat=nx4_registers[`OpenNX4_REG_DRIVERR_BITBANG_TEST][`OpenNX4_REG_DRIVERR_BITBANG_BIT_XLAT];

	always @(negedge pixel_clock) begin
		if (last_sclk != nx4_registers[`OpenNX4_REG_DRIVERR_BITBANG_TEST][`OpenNX4_REG_DRIVERR_BITBANG_BIT_SCLK])begin
			last_sclk=~last_sclk;
			led_sclk<=1;
		end
		else
		begin
			led_sclk<=0;
		end
		
		
	end
/* once we figure out what these do I'm sure we'll want the led driver to wiggle some of them
   inout cpld_p2,
   inout cpld_p3,
   input cpld_p5,
   input cpld_p6,
   inout cpld_p8,
	inout cpld_p41,
   inout cpld_p42,
   inout cpld_p43,
   inout cpld_p44,
*/
`else
	
	parameter INTENSITY_BITSHIFT=6;
	
	reg led_xlat_enable,last_led_xlat_enable;
	//http://www.ti.com/lit/ds/symlink/tlc5941.pdf
	//16 x 12 bit words per driver (3 drivers in series, each 16ch; one R,G,B), so 16*12*3=576 bits per 16-wide column (two columns in parallel)
	
	// if led_mode=1, you're writing dot correction data, 96 bits (6 bits per pixel bigt endian). If Mode=0, you're sending 12 bits per pixel greyscale
	// blanking 0=unblanked, we have to clock a 1 every 4096 greyscale_clocks
	// gsclk is reference clock for pwm grayscale
	
	reg [5:0] word_count;	//48 words per line (3x16)
	reg [3:0] bit_count;		//12 bits per word
	
	reg [2:0] row_count;

	//fuzz unknown pins
   //assign cpld_p2=row_count[2];
   //assign cpld_p3=row_count[1];
   //assign cpld_p8=row_count[0];
   //assign cpld_p5=1'bz;
   //assign cpld_p6=frame_count[2];
	 
	assign cpld_p44=1'bz;  //not reqd to get output
	assign cpld_p43=1'bz; //not reqd to get output
	assign cpld_p42=1'bz; //not reqd to get output
	assign cpld_p41=1'bz; //not reqd to get output

	
	//assign cpld_p8=frame_count[1]; //1; //fuzz_count[5];   //!! required to be wiggled to get output on main leds (but not SIN ones)
	//assign cpld_p5=Z; //fuzz_count[6];  //not reqd to get output
	//	assign cpld_p6=fuzz_count[7];

	parameter GRAYSCALE_BITS=12;	//defined by TI driver
	parameter DOTCORRECT_BITS=6;	//defined by TI driver

	parameter DIE_PER_PIX=3;
	parameter PIXWRDS=3;	//r,g,b
	parameter NUMSCANS=2;
	parameter SCAN_WIDTH=16;
	parameter SCAN_STRIDE=1;
	parameter SCAN_HEIGHT=6;
	parameter NUM_VSEGMENTS=6;
	parameter NUM_SUBPIXELS=((SCAN_WIDTH*NUMSCANS)*PIXWRDS)*NUM_VSEGMENTS;	//total bytes in fb
	parameter SCAN_WORDS=(SCAN_WIDTH*PIXWRDS);	//bytes per half line
	parameter ROW_WORDS=(SCAN_WORDS*NUMSCANS);	//bytes per line
	parameter SCAN_ROW_STEP=(SCAN_WORDS*NUMSCANS*SCAN_HEIGHT);	//bytes per line
	parameter PARALLEL_OUTPUT_BITS=NUM_VSEGMENTS; //=6,   (12 in total, Left and Right)
	parameter SIM_SRC_PIXELS=PARALLEL_OUTPUT_BITS*NUMSCANS; //=12  ;we need to have 12 pixels available per pixel clock so we can slice out the bits. It turns out to be handy that this == GRAYSCALE_BITS, b/c you can just fetch one pixel per output clock in a double buffered setup
	//need parallel access to 12 pixels at a time (and we need to double buffer the fetches)
	reg [GRAYSCALE_BITS-1:0] pixel_buf[0:(SIM_SRC_PIXELS*2)-1]; //we output 6 bits on L and 6 bits on R per cliock, each comes from 12 different pixels in ram, plus we double-buffer the reading (=24)
	
	integer i;	
	//if loading dot correct data or pixels
	wire [3:0] bits_per_word=(led_mode ? (DOTCORRECT_BITS-1) : (GRAYSCALE_BITS-1));

	//Generate source addresses for framebuffer:
	//To provide a 'nice' display pixel ordering in fb memory (r,g,b,r,g,b...) we need to
	//swizzle the bits when we output them.
	//Fetch order for the pixels is as follows;
	//We output 12 pixel streams at once, each is composed of the same bit from (R0..11,G0..11,B0..11)
	//We need to have 12 pixel RGB components in registers at the same time so we can slice the bits off
	//Memory order is R,G,B,R,G,B as a (3x32)x36 linear L->R, T->B 12-bit word array
	//Frame buffer is 1152 words (32x3x36)
	//Read order therefore is;
	//row 0	0  		6(+48)
	//row 6  1(+576)  7(+624)
	//row12  2    		8
   //row18	3 			9
	//row24  4	 	  10
	//row30  5   	  11
	//
	//loopup table for #1 inner loop to avoid a hw multiplier - max value is 2928 (12 bit)
	
	integer agen_pixel_dozen_lookup_output;
	wire [3:0] agen_pixel_dozen_lookup_input;
	always @(agen_pixel_dozen_lookup_input)
	case(agen_pixel_dozen_lookup_input)	//jesus this is a stone-age way to define a lookup table...
		4'd0: agen_pixel_dozen_lookup_output=  0*SCAN_ROW_STEP;
		4'd1: agen_pixel_dozen_lookup_output=  1*SCAN_ROW_STEP;
		4'd2: agen_pixel_dozen_lookup_output=  2*SCAN_ROW_STEP;
		4'd3: agen_pixel_dozen_lookup_output=  3*SCAN_ROW_STEP;
		4'd4: agen_pixel_dozen_lookup_output=  4*SCAN_ROW_STEP;
		4'd5: agen_pixel_dozen_lookup_output=  5*SCAN_ROW_STEP;
		
		4'd6: agen_pixel_dozen_lookup_output=  0*SCAN_ROW_STEP+SCAN_WORDS;
		4'd7: agen_pixel_dozen_lookup_output=  1*SCAN_ROW_STEP+SCAN_WORDS;
		4'd8: agen_pixel_dozen_lookup_output=  2*SCAN_ROW_STEP+SCAN_WORDS;
		4'd9: agen_pixel_dozen_lookup_output=  3*SCAN_ROW_STEP+SCAN_WORDS;
		4'd10: agen_pixel_dozen_lookup_output= 4*SCAN_ROW_STEP+SCAN_WORDS;
		4'd11: agen_pixel_dozen_lookup_output= 5*SCAN_ROW_STEP+SCAN_WORDS;
		default:
				agen_pixel_dozen_lookup_output=1023;	//error condition
	endcase	

	reg [3:0] agen_pixel_dozen_ctr;	//#1 x12 this is the inner loop (0..12, +1 each time); we gather the 12 pixels we need to output in parallel
	reg [5:0] agen_rgb_step_ctr;		//#2 x16 then we do all the R, then G the B from each scan (0..48 +3 each time)
	reg [8:0] agen_scan_step_ctr;		//#3 x6  then we do all scan rows (0..6*(32*3), +(32*3) each time ) 
	reg agen_pixels_ready;
	reg agen_read_buf_index;
	reg agen_write_buf_index;
	reg [5:0] agen_pixel_write_reg;
	reg [1:0] agen_color_ctr;
	assign agen_pixel_dozen_lookup_input=agen_pixel_dozen_ctr;
	
	reg pixel_read_stalled; //debugging


//`define BYPASS_IL   //debug option - turns off 8b->12b lookup and blend multipliers, i.e. output pixels = low 8 bits of FB directly
	reg [11:0] bypass0;
	reg [11:0] bypass1;

	always @(posedge grayscale_clock)
	begin:movePixels //move pixels from framebuffer read through the intensity lookup table via a pre-multiply used for blending/fading
`ifndef BYPASS_IL
			integer il0_taddr,il1_taddr;
			il0_taddr=(fb0_dout[7:0] * nx4_registers[`OpenNX4_REG_FB0_INTENSITY]); 	//address of intensity lookup driven by fb output byte => 12 bit pixel
			il1_taddr=(fb1_dout[7:0] * nx4_registers[`OpenNX4_REG_FB1_INTENSITY]); 	
			il0_addr<=il0_taddr[15 -:8];	
			il1_addr<=il1_taddr[15 -:8];	//using a multiply before the lookup means blends between buffers are a little odd in terms of numbers; could move the multiply to be post-lookup
`else
			bypass0[11:0]<={{0,0,0,0},{fb0_dout[7:0]}};
			bypass1[11:0]<={{0,0,0,0},{fb1_dout[7:0]}};
`endif
	end
	
	// main pixel pushing engine - ti drivers latch on +ve edge of pixel_clock (which we output to them), so we may violate hold time 
	always @(posedge pixel_clock or posedge reset)
	begin 
		if (reset)
		begin
			frame_count<=0;
//`define SKIP_DC_SETUP   //skip loading dotcorrect registers (which takes the first frame) useful if simulating
`ifndef SKIP_DC_SETUP
			led_mode <=1;	//start off in mode 1 (load dot correct) for first frame send
`else
			led_mode <=0; //save time when simulating
`endif			
			vsync<=1;	//to kick things off
			pixel_clock_mask<=0;
		end
		else
		begin
		
			//Generate framebuffer read addresses & fetch our 12 pixels we need to output a full set of bits to the panel shifters
			//the FB is on-chip dual-ported BRAM but could be external (for no obvious gain)
			begin: fb_fetch
				if (vsync) begin
					//first pixel to read after vsync is the last (pixels shifted out in reverse) pixel of the first line
					fb_addr<=(SCAN_WIDTH-1)*PIXWRDS;
					agen_pixel_write_reg<=0;
					row_count<=0;
					led_xlat<=0; //led_xlat_enable <=0;
					led_xlat_enable <=0;
					agen_pixel_dozen_ctr<=0;
					agen_color_ctr<=DIE_PER_PIX-1;
					agen_rgb_step_ctr<=(SCAN_WIDTH-1)*PIXWRDS;
					agen_scan_step_ctr<=0;
					agen_pixels_ready<=0;	//this disables the pixel output engine below, gets turned on after the first pixel batch is fetched, so LED output starts 12 pixel clocks after vsync
					agen_read_buf_index<=0;
					agen_write_buf_index<=0;
					bit_count<=bits_per_word;	//dot correct and grayscale both loaded MSB first
					vsync<=0;
					pixel_clock_mask<=0;
					if (nx4_registers[`OpenNX4_REG_SYSCTL][`OpenNX4_REG_SYSCTL_BIT_UPDATE_DC])begin
						led_mode<=1;	//next frame is dot-correct load
					end
				end
				else
				begin
					if (!agen_pixels_ready || (agen_write_buf_index!=agen_read_buf_index))begin	//this stalls us if the display output consumes pixels slower than we read them (which happens, b/c it needs to insert xlat latch signals per row) 
						//not stalled
						pixel_read_stalled<=0;	//debugging
						if (agen_pixel_dozen_ctr==SIM_SRC_PIXELS-1)	//read 12 pixels (a full set) 
						begin
							agen_write_buf_index<=~agen_write_buf_index;	//there are two sets of 12 pixel registers so we can display one and read the other, so flip that now
							agen_pixel_dozen_ctr<=0;
							
							if (agen_pixels_ready==0)begin	//if main driver wasn't running (we're loading first pixels of a new frame so it's stalled)
								agen_pixels_ready<=1;	//start the main display driver now the first set of pixels are read
								word_count<=SCAN_WORDS-1;
								bit_count<=bits_per_word; //reset its bit counter
							end

							if (agen_rgb_step_ctr==0)
							begin	//done 16 pixels in one color
								agen_rgb_step_ctr<=(SCAN_WIDTH-1)*PIXWRDS;
								if (agen_color_ctr==0)begin	//if done all r,g,b  (16 each)
									agen_color_ctr<=DIE_PER_PIX-1;
									//next scan (of 6)
									if (agen_scan_step_ctr==(SCAN_HEIGHT-1)*ROW_WORDS)
									begin
										//end of frame, although note that we'll get here ahead of the rest of the processing; in this case it'll just wrap around to reading the start of the same FB again (not very usefully). We could make it prefetch the first dozen pixels of the alternate fb, but let's not assume those bytes are ready to display yet
										agen_scan_step_ctr<=0;
									end
									else
									begin
										agen_scan_step_ctr<=agen_scan_step_ctr+ROW_WORDS;
									end
								end
								else
								begin //next color of r,g,b
									agen_color_ctr<=agen_color_ctr-1;
								end
								//
							end
							else
							begin
								agen_rgb_step_ctr<=agen_rgb_step_ctr-PIXWRDS;
							end
						end
						else
						begin
							agen_pixel_dozen_ctr<=agen_pixel_dozen_ctr+1;
						end
						//set up next pixel address
						fb_addr<=agen_pixel_dozen_lookup_output + agen_rgb_step_ctr + agen_scan_step_ctr+agen_color_ctr;
						//track which pixel reg it's destined for
						agen_pixel_write_reg<=agen_pixel_dozen_ctr+(agen_write_buf_index ? 0 : SIM_SRC_PIXELS);
						
						//read the pixels from the previous tick (fb0 and fb1) and add them together
						begin: read_pixel
							integer temp_pixel_out;
							
							if (led_mode==1)begin	//dot correct mode
								temp_pixel_out=nx4_registers[`OpenNX4_REG_DOT_CORRECT_TEST];
								pixel_buf[ agen_pixel_write_reg ][11:0] <= temp_pixel_out[11:0];
							end
							else begin
								//add the two corresponding pixels from each fb from output of 8b->12b lookup
								//they've been pre-multiplied (in 8b space) for buffer fading/blending
//`define OUTPUT_TEST_PIXEL	//debug
`ifndef OUTPUT_TEST_PIXEL
`ifndef BYPASS_IL
								temp_pixel_out[12:0]=il0_dout[11:0] + il1_dout[11:0]; 	//normal operation
`else
								temp_pixel_out[12:0]=bypass0[7:0]+bypass1[7:0];
`endif
`else //test pixel
								temp_pixel_out=nx4_registers[`OpenNX4_REG_TEST_PIXEL];
`endif
								
								if (temp_pixel_out[12])begin //saturate (you can add 100% of each buffer)
									temp_pixel_out = 'hfff; 
								end
								
								//temp_pixel_out=nx4_registers[`OpenNX4_REG_TEST_PIXEL][7:0];
								pixel_buf[ agen_pixel_write_reg ][11:0] <= temp_pixel_out[11:0];
							end
								
						end
					end	//end not stalled
					else
					begin
						pixel_read_stalled<=1; //debugging - note that when led_mode==1 (dot correct; we're only outputting 6 bits per pixel) stalls will occur as the pixels are output faster than they're fetched from the FB.
						//this isn't currently a problem b/c we write a whole 'frame' of dot correct (i.e. 6 rows even though there's only one row to load) and by the time we're done with that it's all propagated in ok
					end
					
					
				end
			end
		
		
		
		
			//led_xlat_enable <= 0; 
			if (led_xlat_enable)begin
				if (led_xlat==0)begin
					led_xlat<=1;
					pixel_clock_mask<=0;	//suppress led pixel clock for latch cycle
				end
				else
				begin
					pixel_clock_mask<=1;
					led_xlat_enable<=0;
					led_xlat<=0;
				end
			end
			else
			begin
				led_xlat<=0;
				//process the read pixels from pixel_buf and extract the bits and sync signals for output to the LED drivers
				if (agen_pixels_ready)
				begin
					pixel_clock_mask<=1;

					if (bit_count== 0 ) //counts down
					begin
						bit_count<=bits_per_word;
						agen_read_buf_index<=~agen_read_buf_index;

						if (word_count == 0)	//16 pixels, each r,g,b
						begin
							led_xlat_enable <= 1;	//latch row
							//pixel_clock_mask<=0;	//suppress led pixel clock for latch cycle

							word_count<=SCAN_WORDS-1;
							
							if (row_count==SCAN_HEIGHT-1)
							begin
								frame_count<=frame_count+1;
								
								if (led_mode==1)//were we in the inital 'load dot correction' mode? if so after the first frame get out of it
								begin
									led_mode<=0;
								end
								agen_pixels_ready<=0;
								vsync<=1;	//this will actually disable this chunk of code b/c the address generator will turn off 'pixels ready' on the next clock and only turn it on when it's fetched the first pixels of the next frame 
								
							end
							else
								row_count <= row_count+1;	
						end
						else
						begin
							word_count <= word_count-1;
						end
					end
					else
					begin
						bit_count <= bit_count-1;
					end
					
					//convert pixels fetched above in pixel_buf by slicing the appropriate bits off
					begin: Serialize
						integer t;
						integer i;

						begin: PixelPacker
							//integer testpixel;
							integer pixel_buf_read_offset;
							assign pixel_buf_read_offset=agen_read_buf_index ? 0 : SIM_SRC_PIXELS;
							//munge the 12 pixels we have into suitable parallel data
							for(t=0;t<PARALLEL_OUTPUT_BITS;t=t+1)begin
								led_l_sin[t+1]<= pixel_buf[(t+pixel_buf_read_offset)&31][bit_count];
								led_r_sin[t+1]<= pixel_buf[(t+pixel_buf_read_offset+PARALLEL_OUTPUT_BITS)&31][bit_count];
							end
							//the cal leds on the back we just drive with the first pixel row of each scan, this isn't really right b/c they're not muxes
							led_cal_sin <=  pixel_buf[(0+pixel_buf_read_offset)&31][bit_count];
						end
					end
					
				end	//pixels ready
			end //xlat check
			
		end  //not reset
	
	end

	always @(*)
	begin
		led_sclk = (~pixel_clock) & pixel_clock_mask; 
	end

`endif

	always @(*)
	begin
		led_gsclk = ~grayscale_clock & ~led_blank;
	end

	always @(posedge grayscale_clock)
	begin
		if (vsync)begin	//blanking/grayscale PWM must be synched to VSync else you get beating effects as the row scan isn't a multiple of the pwm clocks
			blanking_clock <=0;
		end
		else
		begin
			blanking_clock <= blanking_clock+1;
		end
		led_blank <= (blanking_clock==0);
	end
	

endmodule
