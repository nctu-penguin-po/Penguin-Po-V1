#!/usr/bin/env python
# license removed for brevity
import serial
import numpy as np
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray 

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from os import popen


if __name__ == '__main__':
	serial_port = popen('readlink -f /dev/serial/by-path/pci-0000\:00\:14.0-usb-0\:4\:1.0').read().split()[0]
	print(serial_port)
	rospy.init_node('data_node',anonymous=True)
	ser = serial.Serial(serial_port, 9600, timeout = 1)
	#pub1 = rospy.Publisher('posture',Float32MultiArray,queue_size=10)
	pub1 = rospy.Publisher('posture',numpy_msg(Floats),queue_size=10)
	pub2 = rospy.Publisher('depth',Float32,queue_size=10)
	pos = [0, 0, 0]
	while not rospy.is_shutdown():
		a = ser.readline()
		print(a)
		if len(a) < 1:
			continue
		a = a.split()
		
		if len(a) <= 4:
			continue
			
		pos=np.array([float(a[1]),float(a[2]), float(a[3])], dtype = np.float32)
		depth = float(a[4])
		pub1.publish(pos)
		pub2.publish(depth)
