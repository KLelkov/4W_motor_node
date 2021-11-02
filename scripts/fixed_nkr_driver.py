#!/usr/bin/env python

import rospy
import math
import time
import serial # used to exchange data with arduino controller
from std_msgs.msg import String


ser = serial.Serial(
	port='/dev/ttyUSB0',
	baudrate = 115200,
	parity = serial.PARITY_NONE,
	stopbits = serial.STOPBITS_ONE,
	bytesize = serial.EIGHTBITS,
	timeout = 0.5 )

anr1 = 0
anr2 = 0
anr3 = 0
anr4 = 0
turn = 0

stop = 0

def keyboard_callback(data):
    global anr1, anr2, anr3, anr4, turn, stop
    if data.data=="f":
        if anr1 <150:
            stop = 0
            #turn = 0
            anr1 = anr1+5
            anr2 = anr2+5
            anr3 = anr3+5
            anr4 = anr4+5
    if data.data=="b":
        turn = 0
        if anr1>-150:
            anr1 = anr1-5
            anr2 = anr2-5
            anr3 = anr3-5
            anr4 = anr4-5
    if data.data=="l":
        turn =  turn + 1000
        if turn > 7000:
            turn = 7000
        #anr1 = anr1-5
        #anr2 = anr2+5
        #anr3 = anr3-5
       # anr4 = anr4+5
    if data.data=="r":
        turn =  turn - 1000
        if turn  < -7000:
            turn = -7000
        #anr1 = anr1+5
        #anr2 = anr2-5
        #anr3 = anr3+5
        #anr4 = anr4-5

    if data.data=="s":
        stop = 1
        anr1 = 0
        anr2 = 0
        anr3 = 0
        anr4 = 0
    send_to_stm32()

def send_to_stm32():
	global anr1, anr2, anr3, anr4, turn, stop
	ser.write("[drv] {} {} {} {} {} {}\n".format(anr1, int(anr2*1.0), anr3, int(anr4*1.0), turn, int(stop)))
	print('sent: [drv] {} {} {} {} {} {}\n'.format(anr1, anr2, anr3, anr4, turn, int(stop)))
	#print(ser.readline())


def communicator():
    global anr1, anr2, anr3, anr4, turn, stop
    rospy.init_node('nkr_driver_communication', anonymous=True)
    rospy.Subscriber('keyboard_commands', String, keyboard_callback)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        #ser.write("[drv] {} {} {} {} {} {}\n".format(anr1, anr2, anr3, anr4, turn, int(stop)))
        #print('[drv] {} {} {} {} {} {}\n'.format(anr1, anr2, anr3, anr4, turn, int(stop)))
        #print(ser.readline())
        print(ser.readline())


if __name__ == '__main__':
	communicator()
