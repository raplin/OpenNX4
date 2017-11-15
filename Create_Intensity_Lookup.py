"""
//////////////////////////////////////////////////////////////////////////////////
// OpenNX4 - Open source firmware for Barco NX4 tiles
// Company: Bohemian Bits
// Engineer: Richard Aplin (Twitter: @DrTune)
// Copyright (C) 2017 Richard Aplin
// Released under the Attribution-NonCommercial 2.0 Generic (CC BY-NC 2.0)
// You may not use this code or a derived work for commercial purposes unless you have a commercial license, contact drtune@gmail.com
//////////////////////////////////////////////////////////////////////////////////
"""

#output Xilinx ISE  "COE" file (coeffs) to be includes in the ram 8b->12b pixel lookup
fout=open("ipcore_dir/Intensity_Lookup.coe","wt")

#http://jared.geek.nz/2013/feb/linear-led-pwm

res="""
;GENERATED BY CREATE_INTENSITY_LOOKUP.PY DO NOT EDIT
; Generated for a 256x12 dual port BRAM
memory_initialization_radix=16;
memory_initialization_vector=
"""

bits=12
fullScale=1<<bits

def cie1931(L):
    L = L*100.0
    if L <= 8:
        return (L/902.3)
    else:
        return ((L+16.0)/116.0)**3


if True:
    out=[]

    for n in range(256):
        #adjust for  eye response
        if False:
            gamma=pow(fullScale-1,(n/256.0))
            gamma=int(gamma*fullScale)>>bits
        else:
            gamma = round(cie1931(float(n)/255)*(fullScale-1))
            
        if n==0:
            gamma=0
        out.append( gamma )
    res+=(",".join([ "%03x" % n for n in out]))  
    
res+=";\n"

print >>fout,res
print res
fout.close()
