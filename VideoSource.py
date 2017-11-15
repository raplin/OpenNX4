"""
#requires
#ffmpeg
#for hw accel: Armbian legacy kernel with lib-cedarus  (/dev/cedar preset)

mkfifo /tmp/video

#show throughput
cat /tmp/video  | cpipe -vr -vw -vt >/dev/null

solve xwindows problem to allow vdpau decode (else sw)
ffmpeg -hwaccel vdpau -i ../../media/test.mp4 -c:v cedrus264 -f rawvideo -vcodec rawvideo -pix_fmt rgb24 /tmp/video


"""

import time,os,re,sys,threading,Queue

#ignore this bit not using hw accel right now
try:
    with open("/dev/cedar","rb"):
        pass
except:
    pass #print "/dev/cedar not present for hardware acceleration"

FIFO_PIPE="/tmp/videopipe"
try:
    os.mkfifo(FIFO_PIPE)
except:
    pass

QUEUE_LENGTH=4

class VideoSource(object):
    def __init__(self,tileW=32,tileH=36,tileArrayW=3,tileArrayH=3,tileIndexX=1,tileIndexY=1):
        self.lastFrameTime=0

        #total tile array size
        self.tilesX=tileArrayW
        self.tilesY=tileArrayH
        #each tile
        self.tW=tileW
        self.tH=tileH
        #our x,y offset within that
        self.tXOff=tileIndexX*tileW
        self.tYOff=tileIndexY*tileH

        self.outQueue=Queue.Queue(maxsize=QUEUE_LENGTH)

    def start(self,sourceFile,fps=25):
        self.targetFPS=fps
        self.sourceFile=sourceFile
        self.listenerThread=threading.Thread(target=self.startListener)
        self.listenerThread.daemon=True
        self.listenerThread.start()
        self.playerThread=threading.Thread(target=self.startPlayer)
        self.playerThread.daemon=True
        self.playerThread.start()
        

    def startPlayer(self):
        totalWidth,totalHeight=self.tW*self.tilesX , self.tH*self.tilesY
        #cmd="ffmpeg -y -i %s -vf scale=w=%d:h=%d -vf crop=%d:%d:%d:%d -f rawvideo -vcodec rawvideo -pix_fmt rgb24 /tmp/video -nostats -loglevel 0" % (sourceFile,totalWidth,totalHeight,self.tW,self.tH,self.tXOff,self.tYOff)
        #cmd="ffmpeg -y -i %s -vf fps=fps=%d -vf scale=w=%d:h=%d -vf crop=%d:%d:%d:%d -f rawvideo -vcodec rawvideo -pix_fmt rgb24 /tmp/video" % (sourceFile,self.targetFPS,totalWidth,totalHeight,self.tW,self.tH,self.tXOff,self.tYOff)
        quietOption=" -nostats -loglevel 0"
        #cmd="ffmpeg -y -i %s -vf scale=w=%d:h=%d -vf crop=%d:%d:%d:%d -f rawvideo -vcodec rawvideo -pix_fmt rgb24 %s" % (self.sourceFile,totalWidth,totalHeight,self.tW,self.tH,self.tXOff,self.tYOff,FIFO_PIPE)
        #cropping is odd right now
        cmd="ffmpeg -y -i %s -vf scale=w=%d:h=%d -f rawvideo -vcodec rawvideo -pix_fmt rgb24 %s" % (self.sourceFile,totalWidth,totalHeight,FIFO_PIPE)
        print cmd
        cmd+=quietOption
        print "Playing",cmd
        os.system(cmd)

    def readFrame(self,inFile):
        frameSize=3*self.tW*self.tH
        #may stall here
        frame=inFile.read(frameSize)
        #may stall here
        self.outQueue.put(frame)
        
        doPrint=False
        if doPrint:
            for n in range(16):
                print "%02x " % ord(frame[n]),
            print ""
        now=time.time()
        actualFrameTime=now - self.lastFrameTime
        self.lastFrameTime=now
        frameTime=1.0/self.targetFPS
        diff =  frameTime - actualFrameTime
        if diff>0.001:
            #print "sleep",diff
            time.sleep(diff)

    def getDimensions(self):
        return tW,tH

    def nextFrame(self):
        return self.outQueue.get() #blcoks, fps limited

    def startListener(self):
        while True:
            with open(FIFO_PIPE,"rb") as f:
                while True:
                    self.readFrame(f)


def test():
    vs=VideoSource()
    sourceFile="../../media/1080test.mp4"
    vs.start(sourceFile)
    while True:
        frame=vs.outQueue.get()
        print "yay",ord(frame[0])

#test()

