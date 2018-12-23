#!/usr/bin/env python
# license removed for brevity
import numpy as np
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray 
from std_msgs.msg import Int32MultiArray

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
posture_data=[0,0,0]
motor_data=[0,0,0,0,0,0,0,0]
a=1
#consist of motor
Afmd=1650  #first_motor_dive
Asmd=1600  #second_motor_dive
Amf=1500  #motor_float
Amg=1600 #motor_go
Amd=100  #motor_difference
depth_data=0.1
def pos(data):
    global posture_data
    posture_data=data.data
    #print(posture_data)
    #print("posture123456")
    #print(tt.get_num_connections())
def depth_cb(Ddata):
    global depth_data
    depth_data=Ddata

print("posture_data")
if __name__ == '__main__':
    global tt
    rospy.init_node('IMU2motor',anonymous=True)
    pub1 = rospy.Publisher('motor',Int32MultiArray,queue_size=10)
    rate = rospy.Rate(20)
    #print("posture666")

    rospy.Subscriber('posture', numpy_msg(Floats), pos)
    rospy.Subscriber('depth', Float32, depth_cb)

    for i in range(len(motor_data)):
        motor_data[i] = 1500

   
    #rospy.spin()
    while not rospy.is_shutdown():
        a = raw_input()
	a = a.split()
        motor_data[int(a[0])] = int(a[1])
        motorrrr=Int32MultiArray(data = motor_data)
        pub1.publish(motorrrr)
        rate.sleep()
