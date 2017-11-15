import re

class NX4CommsHeaderReaderClass(object):
    def __init__(self,file="nx4_header_file.vh"):
        defines={}
        with open(file,"rt") as f:
            for l in f.readlines():
                m=re.match(r"`define\s+(\w+)\s+(\d+)",l)
                if m:
                    defName=m.group(1)
                    #only simple integers supported!
                    defVal=int(m.group(2))
                    #print defName,defVal
                    self.__dict__[defName]=defVal



                    
NX4=NX4CommsHeaderReaderClass()
