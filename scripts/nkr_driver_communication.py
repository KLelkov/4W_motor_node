#!/usr/bin/env python

import rospy
import math
import time
import serial # used to exchange data with arduino controller
from std_msgs.msg import String
from rdk_msgs.msg import motors


ser = serial.Serial(
	port='/dev/stm',
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


def rps2duty(radpersec):
    duty_cycle = -0.000216 * radpersec**3 + 0.01554 * radpersec**2 + 1.8424 * radpersec + 4.0242
    return int(round(duty_cycle))

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
    if data.data=="r":
        turn =  turn + 5
        if turn > 30:
            turn = 30

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
    odo_pub = rospy.Publisher('motors_data', motors, queue_size=3)
    rate = rospy.Rate(20)
    count = 0
    while not rospy.is_shutdown():
        if count % 2 == 0:
            ser.write("[drv] {} {} {} {} {} {}\n".format(anr1, anr2, anr3, anr4, turn, int(stop)))
        count += 1
        incLine = ser.readline()
        arr = incLine.split(' ')
        if (arr[0] == "[enc]"):
            #print(incLine)
            msg = motors()
            #msg.timestamp = rospy.Time.now()
            msg.odo[0] = float(arr[1])
            msg.odo[1] = float(arr[2])
            msg.odoRear[0] = float(arr[3])
            msg.odoRear[1] = float(arr[4])
            msg.angleFront = float(arr[5])
            msg.angleRear = float(arr[6])
            odo_pub.publish(msg)

        if calibration_needed:
            print('sending rotation angles calibration command\n')
            time.sleep(1)
            ser.write("[cal] \n")
            calibration_needed = False

        rate.sleep()


if __name__ == '__main__':
	communicator()
