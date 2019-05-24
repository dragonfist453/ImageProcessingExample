import rospy, cv2, cv_bridge
import numpy as np
import time
import imutils
import sys
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

class WayPoint:
	
	def __init__(self):

		rospy.init_node('ros_bridge')

		self.ros_bridge = cv_bridge.CvBridge()
		self.image_sub = rospy.Subscriber('/usb_cam/image_rect_color', Image, self.image_callback)

		self.image_pub = rospy.Publisher('/whycon/image_out', Image, queue_size=10)
		self.red = rospy.Publisher('/red', Int32, queue_size=10)
		self.blue = rospy.Publisher('/blue', Int32, queue_size=10)
		self.green = rospy.Publisher('/green', Int32, queue_size=10)

	def image_callback(self,msg):

		image = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

		cx,cy=self.red_count(msg)
		if(cx!=0 and cy!=0):
			cv2.rectangle(image,(cx-20,cy-20),(cx+20,cy+20),(0,0,255),3) #RED
		
		cx,cy=self.green_count(msg)
		if(cx!=0 and cy!=0):	
			cv2.rectangle(image,(cx-20,cy-20),(cx+20,cy+20),(0,255,0),3) #GREEN
	
		cx,cy=self.blue_count(msg)
		if(cx!=0 and cy!=0):
			cv2.rectangle(image,(cx-30,cy-30),(cx+30,cy+30),(255,0,0),3) #BLUE	

		self.image_pub.publish(self.ros_bridge.cv2_to_imgmsg(image, encoding='bgr8'))
		cv2.waitKey(3)
		return 0

	def red_count(self,msg):

		image = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')	
		
		image[:,:,0]=0
		image[:,:,1]=0
		#cv2.imshow('red',image)
		
		hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
		(w,h,c)=hsv.shape
		canvas=np.zeros((w,h),np.uint8)
		canvas=hsv[:,:,2]
		#cv2.imshow('hsvr',canvas)

		ret,thresh=cv2.threshold(canvas,253,254,0)

		image1,contours,hierarchy=cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
		cv2.drawContours(thresh,contours,0,(255,255,255),10)
		image1,contour,hierarchy=cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

		#cv2.imshow('contours',image1)
		cv2.waitKey(3)
		self.red.publish(len(contour))

		if(len(contour)>0):
			cnt=contour[0]
			M = cv2.moments(cnt)
			if(M['m00'] == 0):
				return 0,0
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			#cv2.rectangle(image,(cx-20,cy-20),(cx+20,cy+20),(0,0,255),3)
			return cx,cy
		else:
			return 0,0
			
		

	def green_count(self,msg):

		image = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

		image[:,:,0]=0
		image[:,:,2]=0
		#cv2.imshow('green',image)

		hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
		(w,h,c)=hsv.shape
		canvas=np.zeros((w,h),np.uint8)
		canvas=hsv[:,:,2]
		#cv2.imshow('hsvg',canvas)

		ret,thresh=cv2.threshold(canvas,248,255,0)
		image1,contours,hierarchy=cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
		cv2.drawContours(thresh,contours,0,(255,255,255),5)
		image1,contour,hierarchy=cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

		#cv2.imshow('contours',image1)
		cv2.waitKey(3)
		self.green.publish(len(contour))

		if(len(contour)>0):
			cnt=contour[0]
			M = cv2.moments(cnt)
			if(M['m00'] == 0):
				return 0,0
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			#cv2.rectangle(image,(cx-20,cy-20),(cx+20,cy+20),(0,255,0),3)
			return cx,cy
		else:
			return 0,0
			
		
	def blue_count(self,msg):
		image = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

		image[:,:,1]=0
		image[:,:,2]=0
		#cv2.imshow('blue',image)

		hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
		(w,h,c)=hsv.shape
		canvas=np.zeros((w,h),np.uint8)
		canvas=hsv[:,:,2]
		#cv2.imshow('hsvb',canvas)

		#gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
		ret,thresh=cv2.threshold(canvas,251,255,0)
		image1,contours,hierarchy=cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
		cv2.drawContours(thresh,contours,0,(255,255,255),5)
		image1,contour,hierarchy=cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
		#cv2.imshow('contours',image)

		cv2.waitKey(3)
		self.blue.publish(len(contour))
		if(len(contour)>0):
			cnt=contour[0]
			M = cv2.moments(cnt)
			if(M['m00'] == 0):
				return 0,0
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])	
			return cx,cy
			#cv2.rectangle(image,(cx-30,cy-30),(cx+30,cy+30),(255,0,0),3)
		
		else:
			return 0,0
			
		
if __name__ == '__main__':
	while not rospy.is_shutdown():
		temp1 = WayPoint()
		rospy.spin()


