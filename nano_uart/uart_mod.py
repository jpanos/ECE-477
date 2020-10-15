
#!/usr/bin/python3
import time
import serial
import random
from struct import *
import time

class Flower:
    def __init__(self, name, x, y, z, conf):
        self.name = name
        self.x = x
        self.y = y
        self.z = z
        self.conf = conf

    def __str__(self):
        return "ID: %3d | X: %3d | Y:%3d | Z:%3d | CONF:%3d" % (self.name, self.x, self.y, self.z, self.conf)

testFlowers = []
for i in range(1000):
    testFlowers.append(Flower(i, random.randint(0,256), random.randint(0,256), random.randint(0,256), random.randint(0,100)))
    #testFlowers.append(Flower(0, 1, 2, 3, 4))
print(str(len(testFlowers)) + " test flowers set generated")


NANO_COMM_ON_OFF = b'\x01'
NANO_COMM_ON = b'\x02'
NANO_COMM_OFF = b'\x03'
NANO_COMM_TYPE_FLOWER = b'\x06'

serial_port = serial.Serial(
    port="/dev/ttyTHS1",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)
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
