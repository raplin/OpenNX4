
always begin
`define SPICLK2 100
`define mosi mosi
`define cs cs
`define sck sck
#400 cs=1;
    
// *** AUTOGENERATED DO NOT EDIT *** 

tx_data=8'd138; tx_data_strobe=1; 
#`SPICLK2 `cs=0;
for(i=7;i>=0;i=i-1) begin
#`SPICLK2 `mosi=(104 >> i)&1; `sck=1; #`SPICLK2 `sck=0; 
end
for(i=7;i>=0;i=i-1) begin
#`SPICLK2 `mosi=(101 >> i)&1; `sck=1; #`SPICLK2 `sck=0; 
end
for(i=7;i>=0;i=i-1) begin
#`SPICLK2 `mosi=(108 >> i)&1; `sck=1; #`SPICLK2 `sck=0; 
end
for(i=7;i>=0;i=i-1) begin
#`SPICLK2 `mosi=(108 >> i)&1; `sck=1; #`SPICLK2 `sck=0; 
end
for(i=7;i>=0;i=i-1) begin
#`SPICLK2 `mosi=(111 >> i)&1; `sck=1; #`SPICLK2 `sck=0; 
end
for(i=7;i>=0;i=i-1) begin
#`SPICLK2 `mosi=(32 >> i)&1; `sck=1; #`SPICLK2 `sck=0; 
end
for(i=7;i>=0;i=i-1) begin
#`SPICLK2 `mosi=(116 >> i)&1; `sck=1; #`SPICLK2 `sck=0; 
end
for(i=7;i>=0;i=i-1) begin
#`SPICLK2 `mosi=(104 >> i)&1; `sck=1; #`SPICLK2 `sck=0; 
end
for(i=7;i>=0;i=i-1) begin
#`SPICLK2 `mosi=(101 >> i)&1; `sck=1; #`SPICLK2 `sck=0; 
end
for(i=7;i>=0;i=i-1) begin
#`SPICLK2 `mosi=(114 >> i)&1; `sck=1; #`SPICLK2 `sck=0; 
end
for(i=7;i>=0;i=i-1) begin
#`SPICLK2 `mosi=(101 >> i)&1; `sck=1; #`SPICLK2 `sck=0; 
end
#`SPICLK2 `cs=1;
end
