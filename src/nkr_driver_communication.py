
import math
import time
import serial # used to exchange data with arduino controller



ser = serial.Serial(
	port='/dev/ttyUSB0',
	baudrate = 115200,
	parity = serial.PARITY_NONE,
	stopbits = serial.STOPBITS_ONE,
	bytesize = serial.EIGHTBITS,
	timeout = 0.1)

anr1 = 0
anr2 = 0
anr3 = 0
anr4 = 0
turn = 0
calibration_needed = False
stop = 0
block = 1

anr1a = 0
anr2a = 0
anr3a = 0
anr4a = 0
turna = 0

rw = 0.254/2
lf = 0.4 # forward len
lr = 0.45 # rear len
lw = 0.625 # wheel base (full distsnce between wheels)



def communicator():
    global anr1, anr2, anr3, anr4, turn, stop, calibration_needed, block, anr1a, anr2a, anr3a, anr4a, turna

    count = 0
    while True:
        # if count % 2 == 0:
            # if block == 1:
            #     ser.write("[drv] {} {} {} {} {} {}\n".format(anr1, anr2, anr3, anr4, turn, int(stop)))
            # else:
            #     ser.write("[drv] {} {} {} {} {} {}\n".format(anr1a, anr2a, anr3a, anr4a, turna, 0))
        count += 1
        incLine = ser.readline()
        #arr = incLine.split(' ')

        print(incLine)





if __name__ == '__main__':
	communicator()
