
#!/usr/bin/python3
import time
import serial
import random
from struct import *
import time
import socket

class Flower:
    def __init__(self, regSize):
        self.regSize = regSize
        self.x = []
        self.y = []
        self.z = []

    def loadLoc(self, x, y, z):
        if len(self.x) > self.regSize:
            self.x.pop(0)
            self.y.pop(0)
            self.z.pop(0)
        self.x.append(x)
        self.y.append(y)
        self.z.append(z)

    def getLoc(self):
        return ((sum(self.x)/len(self.x)),(sum(self.y)/len(self.y)),(sum(self.z)/len(self.z)))

    def __str__(self):
        return "ID: %3d | X: %3d | Y:%3d | Z:%3d | CONF:%3d" % (self.name, self.x, self.y, self.z, self.conf)

serial_port = serial.Serial(
    port="/dev/ttyTHS1",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)

def temp():

    testFlowers = []
    for i in range(1000):
        testFlowers.append(Flower(i, random.randint(0,256), random.randint(0,256), random.randint(0,256), random.randint(0,100)))
        #testFlowers.append(Flower(0, 1, 2, 3, 4))
    print(str(len(testFlowers)) + " test flowers set generated")


    NANO_COMM_ON_OFF = b'\x01'
    NANO_COMM_ON = b'\x02'
    NANO_COMM_OFF = b'\x03'
    NANO_COMM_TYPE_FLOWER = b'\x06'


    # Wait a second to let the port initialize
    time.sleep(1)
    try:
        # Send a simple header
        print("waiting for serial port")
        sTime = time.time()
        while serial_port.in_waiting < 1:
            pass
        data = serial_port.read(1)
        if data == NANO_COMM_ON_OFF:
            print("[Received] ON/OFF?")
            serial_port.write(NANO_COMM_ON)
            print("[Sending] ON")
        else:
            print("Unexpected Byte["+ str(data) +"] read from serial port")
        
        while serial_port.in_waiting < 1:
            pass
        data = serial_port.read(1)
        if data == NANO_COMM_TYPE_FLOWER:  
            print("[Received] flower packet request")
            serial_port.write(NANO_COMM_ON)
            print("[Sending] confimation")
        
        while serial_port.in_waiting < 1:
            pass
        temp = serial_port.read(1)
        numFlowers = int.from_bytes(temp, byteorder='big')
        print("[Receivied] "+ str(numFlowers) +" Flower requests")
        print("[Sending] flower data")
        for i in range(numFlowers):
            packet = pack('>iiiii', testFlowers[i].name, testFlowers[i].x, testFlowers[i].y, testFlowers[i].z, testFlowers[i].conf)
            #print("packet = " + str(packet))
            serial_port.write(packet)
        print("[Receiving] flower data")
        while serial_port.in_waiting < 23:
            pass
        print(serial_port.read(23))
        recFlowers = []
        for i in range(numFlowers):
            if serial_port.in_waiting >= 20:
                (a,b,c,d,e) = unpack('>iiiii', serial_port.read(20))
                flower = Flower(a,b,c,d,e)
                print(flower)
                recFlowers.append(flower)
        eTime = time.time()
        print("Done")
        print(f"Protocol took {eTime-sTime} seconds to complete")


    except KeyboardInterrupt:
        print("Exiting Program")

    except Exception as exception_error:
        print("Error occurred. Exiting Program")
        print("Error: " + str(exception_error))

    finally:
        serial_port.close()
        pass


if __name__ == "__main__":
    #connect to serial
    s = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    s.connect("/tmp/flower.socket")
    flowerBuff = Flower(10)
    try:
        while True:
            if serial_port.inWaiting() > 7:
                (x,y) = unpack('<if', serial_port.read(8))
                print(f"receiving UART [{x}][{y}]")
            (a,b,c) = unpack("fff", s.recv(12)) #timing may be weird here, select instead
            flowerBuff.loadLoc(a,b,c) #change ordering?
            loc = flowerBuff.getLoc()
            print(f"sending UART {loc}")
            serial_port.write(pack("<fff", loc[0], loc[1], loc[2])) #change ordering?
    except Exception as e:
        s.shutdown(socket.SHUT_RDWR)
        s.close()
        print("exiting with exception + " + str(e))
    #connect to UART

    
