OpenNX4
========

Open firmware for the Barco NX4 LED video tile.

About
-------------
The Barco NX4 is a high quality 32x36 pixel RGB LED tile, intended for use in sets of 9 (3x3) with a controller/power supply. These sets in turn are used as part of an arbitrarily large video wall, each 3x3-tile controller takes a video input (using HDMI cabling but not protocol) from an expensive looking rackmount video controller.

The tiles have been/are available on ebay at reasonable prices, but the controllers are not. The tiles are rather high quality devices; they use 1/6 scan with 12-bit constant current drivers, have a nice fine dot pitch (4mm) and use high quality LEDs with black filler inside to improve contrast. 

Each tile uses a Xilinx FPGA (Spartan 3E / 250) to drive the pixels, and has an IN and OUT ports for daisychaining, supplying data (LVDS pairs) and power (24v DC). There is no CPU, but it's possible that the stock Barco FPGA bitstream contains a soft-cpu core.  The FPGA boots off a flash memory, and has a readily accessible JTAG port. There is an external SRAM and plenty of spare flash on the tile.

The author does not have access to any other Barco hardware except the tile, so decided not to bother trying to blind-guess Barco's data format to load pixels to the tile, and instead start afresh.

Because the tile is FPGA driven, it is a remarkably flexible platform, as we shall see.

Goals
--------
This project provides alternate firmware for the Xilinx FPGA that turns the tile into a multi-format display device, suitable for
* Displaying >60fps video from a ~$15 Orange Pi board (one of which can drive many daisychained tiles; likely >10, TBC)
* Pretending to be a string of 32x36 (1152) WS2812B NeoPixels
* Receiving pixels (frame buffer, bitmaps) over UART or SPI from an Arduino or similar (I2C is possible)

Additionally the OpenNX4 firmware has support for
* Dual frame buffers (load one while the other displays), plus programmable blending between the two buffers
* A programmable 8bit->12bit lookup table for converting the frame buffer to the 12-bit LED driver format.  (8 bits per pixel is perfectly sufficient if CIE intensity correction is performed on the tile)
* ...much more is possible, the FPGA is fairly empty still...


Resources
--------
Much project info can be found on two other resources;
The [Hackaday Project page](https://hackaday.io/project/27799-barco-nx-4-reversing-adventure) is a the main resource, and is where you should go to read much more about this tile, how to program it, and meet some great folks.
A [Google doc](https://docs.google.com/document/d/1jUVEgcwudxltnb_0Td4SBvetes3Vr6FILnSM_tLnx6o/edit?usp=sharing) I started (older, tech notes)
There is also [a bunch of PCB pr0n](https://drive.google.com/drive/folders/0B78sx1JgISTYa2lyWG83ejhvX1E?usp=sharing)

Current status
-------------
####What works
* LED driving
    * display scan rate of 1438fps ;-)
    * BUT right now we don't know how the 1/6 row mux works (see below)
* SRAM and Flash accesses
    * A few timing bugs with SRAM at present but it's not used (the frame buffers are dual-port memory in the FPGA, there's plenty of space)
* Communications
    * UART works (Mbits possible)
    * SPI, WS2812B partially implemented, needs debugging
    * I2C is working (host bit-bangs it currently)
* Host interface
    * From a python script (over a UART currently) you can send commands to 
        * read/write the internal registers (16 registers)
        * write to the frame buffer(s), 8b-12b lookup table, SRAM and Flash
        

### What doesn't work 
* Minor I2C stuff like the onboard temperature and light sensors; can talk to them over i2c, just didn't write (or borrow) a driver yet.
* Haven't tried reprogramming the Xilinx image in the onboard flash yet (should be fine, the .bit file just goes at the start of the flash)
* BIG >>  Row scanning:
The tile uses a 1/6 row mux, which is very good (i.e. bright, low flicker/artifacts) BUT we don't currently understand how to switch rows.

...Well, we do, it's driven by the CPLD on the LED display board, but we don't know how to get the CPLD to switch to a new row. This can't be hard; likely the CPLD is currently in a dormant state and needs the right signalling to get it going.
* What would be handy would be a logic analyzer capture of the Xilinx -> CPLD pins from a running (displaying video) tile.
* ...or for someone to just fuzz it (most easily from the Python code sending bit-bang instructions to the FPGA to wiggle the CPLD i/o ports) and get lucky.

Todos
-----
There is an extensive list of fun things that can/will be added ONCE THE ROW SCAN IS FIGURED OUT;
* High speed SPI video (incl daisychain) - will do this as soon as we figure out row scanning
* frame generator/video head using an Orange Pi (for bonus points, use both I2S ports synchonously to get ~100mbps), can take an image (e.g. H264 video, remote desktop etc), format it, and output to a chain of tiles.
* various fixes; there are bugs in the SRAM write, the WS2812B and SPI stuff needs finishing
* additions to help lower-speed hosts like ardunios (e.g. load several images into SRAM in advance then send commands to blend through them, use them as sprites, scroll the tile contents, etc) - basically turn it into a video controller+display
* put a soft-6502 in there and emulate a Nintendo NES, including 1-bit delta sigma audio out from the i/o port


Requirements
---------
* If you want to build the FPGA image you want the Xilinx ISE webpack (free). Don't get Vivaldo, get the last release of ISE.
    * OR there is also a precompiled bitstream (.bit) and equivalent .SVF file (a simple format supported by many JTAG programmers) in the git repo
* A 3v3 UART (e.g. FTDI dongle or whatever) - 5v will not do. 
    * use some 100ohm (ish) resistors in series with the UART also to prevent badness; the FPGA runs below 3v3
* Python 2
    * plus pyserial (optionally pygame) (windows, linux or mac hosts should all work)
* A 12-24v DC power supply (the tile can suck ~40 watts at full wack)
* A JTAG adapter with level shifting; the Xilinx JTAG runs at 2.8v.
    * See the hackaday page for more info about jtag etc


Howto
----
This is yet to be written. 
Quick hint; 
* connect your:
    * 3v3 UART TX via a 100ohm resistor to Pin 4 on the input (male) connector, 
    * UART RX (data from tile) to Pin 5
    * GND to (whatever the gnd is, see hackaday page). 
* power up the tile
* see !Precompiled_Binaries\V0.1
* load the "test_pattern_boot.svf" bitstream. This should make wild stripy colors appear. 
    * This just inits some stuff (this is a hack and will be fixed; you only need to load this image the first time the tile is powered up)
* then load the "toplevel.bit" (or .svf) file
* the run "python NX4Comms.py"  (may need to tweak it to locate your COM port)
* ...should display an image but with only one out of each six rows lit up.
* for more playing, see the python. You can bit-bang the CPLD lines from that python code, see if you can figure out how to get the row scanning working


### Caution; don't pop your FPGA: 
Do not use a 5v UART - use a 3v3 with ~100ohm resistors. 
Do not use a 3v3 JTAG interface, use one with a proper level shifter output and hook that to 2.8v on the board (see hackaday page)


### Credits
Initial Verilog projects and python stuff by Richard Aplin (@DrTune), with help from and thanks to the the guys on the hackaday project page. ( Ian Hanschen , modder_mike )
