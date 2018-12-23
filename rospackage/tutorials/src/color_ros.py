#!/usr/bin/env python
# license removed for brevity
import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray 

if __name__ == '__main__':
	rospy.init_node('color',anonymous=True)
	pub1 = rospy.Publisher('colorpy',String,queue_size=10)
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
		x = [None for i in range(2)]
		y = [None for i in range(2)]
		w = [None for i in range(2)]
		h = [None for i in range(2)]
		Xlabal = [None for i in range(2)]
		Ylabal = [None for i in range(2)]
		colorx = [None for i in range(2)]
		color = ['blue','yellow']
		
		
		#up & low
		#blue
		lower[0] = np.array([110,10,10])
		upper[0] = np.array([130,255,255])
		
		#yellow
		lower[1] = np.array([20,100,100])
		upper[1] = np.array([40,200,200])

		
		for i in range(len(lower)):
			#binary
			mask[i] = cv2.inRange(hsv,lower[i],upper[i])
			#blur
			blur[i] = cv2.medianBlur(mask[i],7)
			#find contours
			(_,contours[i],_) = cv2.findContours(blur[i],cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

			#when find contours
			if len(contours[i]) > 0:
				#max
				contour[i] = max(contours[i],key=cv2.contourArea)
				#rec
				rect[i] = np.int32(cv2.boxPoints(cv2.minAreaRect(contour[i])))
				#labal
				x[i],y[i],w[i],h[i] = cv2.boundingRect(rect[i])		
				if (w[i]+h[i]) > 300:
					#draw contours
					cv2.drawContours(img,[rect[i]],-1,(0,0,255),2)
					Xlabal[i] = (x[i]+x[i]+w[i])/2
					Ylabal[i] = (y[i]+y[i]+h[i])/2
					#labal = [Xlabal[i],Ylabal[i]]
					colorx[i] = " ".join(map(str, [color[i], Xlabal[i], Ylabal[i]]))
					l = color[i].split(" ")# l is a list split by space
					pub1.publish(colorx[i])
					rate.sleep()


		
		cv2.imshow('image',img)



		
		
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	cap.release()
	cv2.destroyAllWindows()
