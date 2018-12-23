#!/usr/bin/env python
# license removed for brevity
import cv2
import numpy as np
import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray 
from geometry_msgs.msg import Point

if __name__ == '__main__':
	rospy.init_node('color',anonymous=True)
	pub1 = rospy.Publisher('Bcolorpy',String,queue_size=10)
	pub2 = rospy.Publisher('Bdistantpy',Point,queue_size=10)
	pub3 = rospy.Publisher('Ycolorpy',String,queue_size=10)
	pub4 = rospy.Publisher('Ydistantpy',Point,queue_size=10)
	#pub2 = rospy.Publisher('labal',Float32MultiArray,queue_size=10)
	#pub3 = rospy.Publisher('labal',Float32,queue_size=1)
	rate = rospy.Rate(20)
	img = cv2.imread('')
	flag_im = False

	#cap = cv2.VideoCapture(0)
	'''
	cap = cv2.VideoCapture(0)
	flag_im = False
	'''
	def imSub(imgmsg):
	  global img, flag_im
	  bridge = CvBridge()
	  img = bridge.imgmsg_to_cv2(imgmsg, 'bgr8')

	rospy.Subscriber("usb_cam/image_raw", Image, imSub, queue_size=1, buff_size=2**24)
	#rospy.Subscriber("/camera/image_raw", Image, imSub, queue_size=1, buff_size=2**24)
	for i in range(0,100):
		print("spin callback")
		rate.sleep()

	while not rospy.is_shutdown():
		#ret,img = cap.read()
		
		#BGR to HSV
		hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
		#List
		empty = [None for i in range(2)]
		
		lower = [None for i in range(2)]
		upper = [None for i in range(2)]
		mask = [None for i in range(2)]
		blur = [None for i in range(2)]
		contours = [None for i in range(2)]
		contour = [None for i in range(2)]
		rect = [None for i in range(2)]
		X = [None for i in range(2)]
		Y = [None for i in range(2)]
		W = [None for i in range(2)]
		H = [None for i in range(2)]
		Xlabal = [None for i in range(2)]
		Ylabal = [None for i in range(2)]
		EXlabal = [None for i in range(2)]
		EYlabal = [None for i in range(2)]
		Ax = [None for i in range(2)]
		Ay = [None for i in range(2)]
		Aw = [None for i in range(2)]
		Px = [None for i in range(2)]
		Py = [None for i in range(2)]
		Pz = [None for i in range(2)]
		colorx = [None for i in range(2)]
		distant = [None for i in range(2)]
		D=[0,0,0]
		color = ['blue','yellow']
		#nx=36*np.pi/180
		#ny=27*np.pi/180
		nx=20*np.pi/180
		ny=15*np.pi/180
		Pw=20.2
		#Pw=4.2
		
		
		
		#up & low
		#blue
		lower[0] = np.array([100,40,45])
		upper[0] = np.array([130,255,255])

		
		#yellow
		lower[1] = np.array([20,100,100])
		upper[1] = np.array([40,200,200])

		
		
		#binary
		mask[0] = cv2.inRange(hsv,lower[0],upper[0])
		#blur
		blur[0] = cv2.medianBlur(mask[0],7)
		#find contours
		(_,contours[0],_) = cv2.findContours(blur[0],cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

		#when find contours
		if len(contours[0]) > 0:
				#max
				contour[0] = max(contours[0],key=cv2.contourArea)
				#rec
				rect[0] = np.int32(cv2.boxPoints(cv2.minAreaRect(contour[0])))
				#print(rect[0][0])
				#labal
				X[0],Y[0],W[0],H[0] = cv2.boundingRect(rect[0])		
				if (W[0]+H[0]) > 100:
					#draw contours
					cv2.drawContours(img,[rect[0]],-1,(0,0,255),2)
					Xlabal[0] = (X[0]+X[0]+W[0])/2
					Ylabal[0] = (Y[0]+Y[0]+H[0])/2
					EXlabal[0]=Xlabal[0]-320
					EYlabal[0]=240-Ylabal[0]
					Ax[0]=EXlabal[0]*nx/320
					Ay[0]=EYlabal[0]*ny/240
					Aw[0]=W[0]*nx/320
					Pz[0]=Pw/np.tan(Aw[0])
					Px[0]=np.tan(Ax[0])*Pz[0]
					Py[0]=np.tan(Ay[0])*Pz[0]	
					#labal = [Xlabal[i],Ylabal[i]]
					x=Px[0]
					y=Py[0]
					z=Pz[0]

					colorx[0] = " ".join(map(str, [color[0], Xlabal[0], Ylabal[0]]))
					distant[0] = " ".join(map(str, [color[0], Px[0], Py[0], Pz[0]]))	
					D=[Px[0],Py[0],Pz[0]]
					pub1.publish(colorx[0])
					pub2.publish(Px[0],Py[0],Pz[0])
		
		#binary
		mask[1] = cv2.inRange(hsv,lower[1],upper[1])
		#blur
		blur[1] = cv2.medianBlur(mask[1],7)
		#find contours
		(_,contours[1],_) = cv2.findContours(blur[1],cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

		#when find contours
		if len(contours[1]) > 0:
				#max
				contour[1] = max(contours[1],key=cv2.contourArea)
				#rec
				rect[1] = np.int32(cv2.boxPoints(cv2.minAreaRect(contour[1])))
				#labal
				X[1],Y[1],W[1],H[1] = cv2.boundingRect(rect[1])		
				if (W[1]+H[1]) > 300:
					#draw contours
					cv2.drawContours(img,[rect[1]],-1,(0,0,255),2)
					Xlabal[1] = (X[1]+X[1]+W[1])/2
					Ylabal[1] = (Y[1]+Y[1]+H[1])/2
					EXlabal[1]=Xlabal[1]-320
					EYlabal[1]=240-Ylabal[1]
					Ax[1]=EXlabal[1]*nx/320
					Ay[1]=EYlabal[1]*ny/240
					Aw[1]=W[1]*nx/320
					Pz[1]=Pw/np.tan(Aw[1])
					Px[1]=np.tan(Ax[1])*Pz[1]
					Py[1]=np.tan(Ay[1])*Pz[1]

					#labal = [Xlabal[i],Ylabal[i]]
					colorx[1] = " ".join(map(str, [color[1], Xlabal[1], Ylabal[1]]))
					distant[1] = " ".join(map(str, [color[1], Px[1], Py[1], Pz[1]]))
					#l = color[i].split(" ")# l is a list split by space
					pub3.publish(colorx[1])
					pub4.publish(Px[1],Py[1],Pz[1])
					rate.sleep()


		
		cv2.imshow('image',img)



		
		
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	cap.release()
	cv2.destroyAllWindows()
