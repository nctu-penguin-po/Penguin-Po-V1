#!/usr/bin/env python
# license removed for brevity
import numpy as np
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray 
from std_msgs.msg import Int32MultiArray
import time
import serial
from os import popen

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
motor_data=[1500,1500,1500,1500,1500,1500,1500,1500]
def motor_cb(mdata):
    print('       qqq')
    for i in range(0,8):
        motor_data[i]=mdata.data[i]

if __name__ == '__main__':
    serial_port = popen('readlink -f /dev/serial/by-path/pci-0000\:00\:14.0-usb-0\:3\:1.0').read().split()[0]
    rospy.init_node('arduino_download',anonymous=True)
    rospy.Subscriber('motor', Int32MultiArray, motor_cb)
    ser = serial.Serial(serial_port, 115200, timeout = 1)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        for i in range(len(motor_data)):
            ser.write(chr(motor_data[i]/10))
            #print(motor_data[i]/10)
        ser.write(chr(50))
        a = ser.read(1000)
        print(a)
        #print(motor_data[0])
        time.sleep(0.1)
    print('end')
