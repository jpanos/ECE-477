
#!/usr/bin/python3
import time
import serial
import random
from struct import *
import time
import socket
import math
from termcolor import colored

class Flower:
    def __init__(self, regSize):
        self.regSize = regSize
        self.zSize = 2
        self.active = False
        self.x = []
        self.y = []
        self.z = []

    def loadLoc(self, x, y, z):
        if len(self.x) > self.regSize:
            self.x.pop(0)
            self.y.pop(0)
        if len(self.z) > self.zSize:
            self.z.pop(0)
        self.x.append(x)
        self.y.append(y)
        self.z.append(z)

    def getEuclidean(self):
        loc = self.getLoc()
        if (not self.active) and (abs(loc[0]) < 50):
            self.active = True
            print(colored("[NANO][INFO ] LOCATED FLOWER CLUSTER","blue"));
        return (math.sqrt((loc[0]*loc[0] + loc[1]*loc[1] + loc[2]*loc[2])))

    def getLoc(self):
        #return ((sum(self.x)/len(self.x)),(sum(self.y)/len(self.y)),(sum(self.z)/len(self.z)))
        return ((sum(self.x)/len(self.x)),(sum(self.y)/len(self.y)),(sum(self.z)/len(self.z)))

    

serial_port = serial.Serial(
    port="/dev/ttyTHS1",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)

if __name__ == "__main__":
    #connect to serial
    s = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    s.connect("/tmp/flower.socket")
    flowerBuff = Flower(5)
    bat = 0.0
    per = 0
    try:
        tStart = time.time()
        while True:
            while serial_port.inWaiting() > 7:
                (per, bat) = unpack('<if', serial_port.read(8))
                print(colored(f"[NANO][RX   ]                                                    [{per}%][{bat:.3f}V]", "green"))
            while serial_port.inWaiting() > 0:
                print(colored(f"[NANO][RX   ]Flushing extra bytes [{serial_port.read(1)}]", "blue"))
            (a,b,c) = unpack("fff", s.recv(12)) #timing may be weird here, select instead
            flowerBuff.loadLoc(a,b,c) #change ordering?
            if time.time() - tStart > 0.10:
                tStart = time.time()
                loc = flowerBuff.getLoc()
                dist = flowerBuff.getEuclidean()
                print(colored(f"[NANO][DIST ][[{loc[0]:09.1f}][{loc[1]:09.1f}][{loc[2]:09.1f}]]:{dist:09.1f}]", "yellow") + colored(f"[INFO][{per}%][{bat:.3f}V]","blue"))
                if dist < 2450:
                    print(colored(f"[NANO][TX   ][[{loc[0]:09.1f}][{loc[1]:09.1f}][{loc[2]:09.1f}]]", "red"))
                    serial_port.write(pack("<fff", loc[0], loc[1], loc[2])) #change ordering?
    except Exception as e:
        s.shutdown(socket.SHUT_RDWR)
        s.close()
        print("exiting with exception + " + str(e))
    #connect to UART

    
