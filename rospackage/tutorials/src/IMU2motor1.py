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

#consist of motor
Afmd=1800  #first_motor_dive
Asmd=1750  #second_motor_dive
Amf=1700  #motor_float
Amg=1700 #motor_go
Amd=25  #motor_difference
Amsf = 1500 #motor_side_front
Amsb = 1500 #motor_side_behinde

depth_data=0.1
a=1
def pos(data):
    global posture_data
    posture_data=data.data
    #print(posture_data)
    #print("posture123456")
    #print(tt.get_num_connections())
def depth_cb(Ddata):
    global depth_data
    depth_data=Ddata.data

print("posture_data")
if __name__ == '__main__':
    global tt
    rospy.init_node('IMU2motor',anonymous=True)
    pub1 = rospy.Publisher('motor',Int32MultiArray,queue_size=10)
    rate = rospy.Rate(20)
    #print("posture666")

    rospy.Subscriber('posture', numpy_msg(Floats), pos)
    rospy.Subscriber('depth', Float32, depth_cb)


   
    #rospy.spin()
    while not rospy.is_shutdown():
        print(posture_data)
        print(a)
        print(depth_data)
        if a==1:
            if depth_data<0.1:
                motor_data=[1500,1500,1500,1500,Afmd,Afmd,Afmd,Afmd]
            elif depth_data > 0.1:
                a=0
        else:
            if depth_data<0.1: #depth
                if posture_data[1]<-5:     #row right
                    if posture_data[2]>-175: #pitch down
                        motor_data=[Amsf,Amsb,Amg,Amg,Asmd,Asmd+Amd,Asmd-Amd,Asmd]

                    elif posture_data[2]<175:                  #pitch up
                        motor_data=[Amsf,Amsb,Amg,Amg,Asmd+Amd,Asmd,Asmd,Asmd-Amd]

                    elif posture_data[2]<=-175 and posture_data[2]>=175 #pitch remain
                        motor_data=[Amsf,Amsb,Amg,Amg,Asmd+Amd,Asmd+Amd,Asmd-Amd,Asmd-Amd]					

                elif posture_data[1]>5:                      #row left
                    if posture_data[2]>-175:  #pitch down
                        motor_data=[Amsf,Amsb,Amg,Amg,Asmd-Amd,Asmd,Asmd,Asmd+Amd]

                    elif posture_data[2]<175:                  #pitch up
                        motor_data=[Amsf,Amsb,Amg,Amg,Asmd,Asmd-Amd,Asmd+Amd,Asmd]

                    elif posture_data[2]<=-175 and posture_data[2]>=175 #pitch remain
                        motor_data=[Amsf,Amsb,Amg,Amg,Asmd-Amd,Asmd-Amd,Asmd+Amd,Asmd+Amd]	
						
                elif posture_data[1] >= -5 and posture_data[1] <= 5:  #row remain
                    if posture_data[2]>-175: #pitch down
                        motor_data=[Amsf,Amsb,Amg,Amg,Asmd-Amd,Asmd+Amd,Asmd-Amd,Asmd+Amd]

                    elif posture_data[2]<175:                  #pitch up
                        motor_data=[Amsf,Amsb,Amg,Amg,Asmd+Amd,Asmd-Amd,Asmd+Amd,Asmd-Amd]

                    elif posture_data[2]<=-175 and posture_data[2]>=175 #pitch remain
                        motor_data=[Amsf,Amsb,Amg,Amg,Asmd,Asmd,Asmd,Asmd]					

            elif depth_data > 0.5:
                if posture_data[1]<-5:     #row right
                    if posture_data[2]>-175: #pitch down
                        motor_data=[Amsf,Amsb,Amg,Amg,Amf,Amf+Amd,Amf-Amd,Amf]

                    elif posture_data[2]<175:                 #pitch up
                        motor_data=[Amsf,Amsb,Amg,Amg,Amf+Amd,Amf,Amf,Amf-Amd]

                    elif posture_data[2]<=-175 and posture_data[2]>=175 #pitch remain
					    motor_data = [Amsf,Amsb,Amg,Amg,Amf+Amd,Amf+Amd,Amf-Amd,Amf-Amd]

                elif posture_data[1]>5:                      #row left
                    if posture_data[2]>-175:  #pitch down
                        motor_data=[Amsf,Amsb,Amg,Amg,Amf-Amd,Amf,Amf,Amf+Amd]

                    elif posture_data[2]<175:                  #pitch up
                        motor_data=[Amsf,Amsb,Amg,Amg,Amf,Amf-Amd,Amf+Amd,Amf]

                    elif posture_data[2]<=-175 and posture_data[2]>=175 #pitch remain
					    motor_data = [Amsf, Amsb, Amg, Amg, Amf-Amd, Amf-Amd, Amf+Amd, Amf+Amd]

                            elif posture_data[1] >= -5 and posture_data[1] <= 5:  #row remain
                    if posture_data[2]>-175: #pitch down
                        motor_data=[Amsf,Amsb,Amg,Amg,Amf-Amd,Amf+Amd,Amf-Amd,Amf+Amd]

                    elif posture_data[2]<175:                  #pitch up
                        motor_data=[Amsf,Amsb,Amg,Amg,Amf+Amd,Amf-Amd,Amf+Amd,Amf-Amd]

                    elif posture_data[2]<=-175 and posture_data[2]>=175 #pitch remain
                        motor_data=[Amsf,Amsb,Amg,Amg,Amf,Amf,Amf,Amf]   
						
        motorrrr=Int32MultiArray(data = motor_data)
        pub1.publish(motorrrr)
        rate.sleep()
