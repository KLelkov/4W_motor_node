#!/usr/bin/env python

import rospy
import math
import time
import serial # used to exchange data with arduino controller
from std_msgs.msg import String
from rdk_msgs.msg import motors
from rdk_msgs.msg import control


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


def rps2duty(radpersec):
    duty_cycle = math.copysign(-0.000216 * abs(radpersec)**3 + 0.01554 * radpersec**2 + 1.8424 * abs(radpersec) + 4.0242, radpersec)
    if abs(duty_cycle) < 8:
        duty_cycle = 0
    elif abs(duty_cycle) > 80:
        duty_cycle = math.copysign(80, duty_cycle)
    return int(round(duty_cycle))


def velocity2commands(velocity, rate):
    if velocity == 0: # cant turn on place (without moving forward)
        if rate != 0:
            gamma = math.copysign(25, rate)
            return (8, 8, 8, 8, round(gamma))
        else:
            return (0, 0, 0, 0, 0)
    else:
        tan_gamma = rate * (lf + lr) / abs(velocity) / 2
        gamma = math.atan(tan_gamma) * 180.0 / math.pi
        if abs(gamma) > 30:
            gamma = math.copysign(30, gamma)
    if  rate == 0:
        duty1 = rps2duty(velocity / rw)
        duty2 = rps2duty(velocity / rw)
        duty3 = rps2duty(velocity / rw)
        duty4 = rps2duty(velocity / rw)
    else: # if turning - velocities of the outer wheels should be higher
        Rc = abs(velocity / rate)
        vel1 = abs(rate) * math.sqrt( lf**2 + (Rc + math.copysign(lw/2, rate))**2 ) * math.copysign(1, velocity)
        vel2 = abs(rate) * math.sqrt( lf**2 + (Rc - math.copysign(lw/2, rate))**2 ) * math.copysign(1, velocity)
        vel3 = abs(rate) * math.sqrt( lr**2 + (Rc + math.copysign(lw/2, rate))**2 ) * math.copysign(1, velocity)
        vel4 = abs(rate) * math.sqrt( lr**2 + (Rc - math.copysign(lw/2, rate))**2 ) * math.copysign(1, velocity)
        duty1 = rps2duty(vel1 / rw)
        duty2 = rps2duty(vel2 / rw)
        duty3 = rps2duty(vel3 / rw)
        duty4 = rps2duty(vel4 / rw)
    return (duty1, duty2, duty3, duty4, round(gamma))


def keyboard_callback(data):
    global anr1, anr2, anr3, anr4, turn, stop, calibration_needed, block
    if data.data == "block":
        block = abs(int(block - 1))
        if block == 1:
            print("manual control mode enabled.")
        else:
            print("Automatic control mode!")
    if data.data=="f":
        if anr1 <75:
            stop = 0
            anr1 = anr1+5
            anr2 = anr2+5
            anr3 = anr3+5
            anr4 = anr4+5
    if data.data=="b":
        if anr1>-75:
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


def control_callback(data):
    global anr1a, anr2a, anr3a, anr4a, turna
    time_diff = rospy.get_time() * 1000 - data.timestamp
    if abs(time_diff) < 1000:
        anr1a, anr2a, anr3a, anr4a, turna = velocity2commands(data.ups, data.dth)
        #ser.write("[drv] {} {} {} {} {} {}\n".format(w1, w2, w3, w4, gamma, 0))


def communicator():
    global anr1, anr2, anr3, anr4, turn, stop, calibration_needed, block, anr1a, anr2a, anr3a, anr4a, turna
    rospy.init_node('nkr_driver_communication', anonymous=True)
    rospy.Subscriber('keyboard_commands', String, keyboard_callback)
    rospy.Subscriber('control_commands', control, control_callback)
    odo_pub = rospy.Publisher('motors_data', motors, queue_size=3)
    rate = rospy.Rate(20)
    count = 0
    while not rospy.is_shutdown():
        if count % 2 == 0:
            if block == 1:
                ser.write("[drv] {} {} {} {} {} {}\n".format(anr1, anr2, anr3, anr4, turn, int(stop)))
            else:
                ser.write("[drv] {} {} {} {} {} {}\n".format(anr1a, anr2a, anr3a, anr4a, turna, 0))
        count += 1
        incLine = ser.readline()
        arr = incLine.split(' ')
        if (arr[0] == "[enc]"):
            if count % 50 == 0:
                print(incLine)
            msg = motors()
            msg.timestamp = rospy.get_time() * 1000
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
