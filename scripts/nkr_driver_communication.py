#!/usr/bin/env python

import rospy
import math
import time
import serial # used to exchange data with arduino controller
from std_msgs.msg import String


ser = serial.Serial(
	port='/dev/ttyUSB1',
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

def keyboard_callback(data):
    global anr1, anr2, anr3, anr4, turn, stop, calibration_needed
    if data.data=="f":
        if anr1 <100:
            stop = 0
            anr1 = anr1+5
            anr2 = anr2+5
            anr3 = anr3+5
            anr4 = anr4+5
    if data.data=="b":
        if anr1>-100:
            stop = 0
            anr1 = anr1-5
            anr2 = anr2-5
            anr3 = anr3-5
            anr4 = anr4-5
    if data.data=="l":
        turn =  turn - 5
        if turn  < -30:
            turn = -30
        #anr1 = anr1-5
        #anr2 = anr2+5
        #anr3 = anr3-5
        #anr4 = anr4+5
    if data.data=="r":
        turn =  turn + 5
        if turn > 30:
            turn = 30
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
    if data.data=="o": # calibration command
        stop = 1
        turn = 0
        anr1 = 0
        anr2 = 0
        anr3 = 0
        anr4 = 0
        calibration_needed = True




def communicator():
    global anr1, anr2, anr3, anr4, turn, stop, calibration_needed
    rospy.init_node('nkr_driver_communication', anonymous=True)
    rospy.Subscriber('keyboard_commands', String, keyboard_callback)
    rate = rospy.Rate(8)

    while not rospy.is_shutdown():
        ser.write("[drv] {} {} {} {} {} {}\n".format(anr1, anr2, anr3, anr4, turn, int(stop)))
        #print('[drv] {} {} {} {} {} {}\n'.format(anr1, anr2, anr3, anr4, turn, int(stop)))
        #print(ser.readline())
        print(ser.readline())

        if calibration_needed:
            print('sending rotation angles calibration command\n')
            time.sleep(1)
            ser.write("[cal] \n")
            calibration_needed = False

        rate.sleep()


if __name__ == '__main__':
	communicator()
