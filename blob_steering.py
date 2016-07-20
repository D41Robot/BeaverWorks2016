#!/usr/bin/env python
import cv2
import rospy
import math
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped

class blob_steering:
	
	Kp=.5
	Kd=0
	Ki=0
	def __init__(self):
		rospy.Subscriber("image_size", Float32)
		rospy.Subscriber("imagepos_error", Float32, self.calcdrive)

		self.pub = rospy.Publisher('/racecar/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=5)

	def drive(self, angle, speed):

        	ackmsg = AckermannDriveStamped()
       		ackmsg.drive.speed = speed
        	ackmsg.drive.steering_angle = angle

        	self.pub.publish(ackmsg)
		#print "published"
	
	def calcdrive(self, msg):
		#float f = float
		#print str(type(msg))
		if msg>0:
			self.drive(.5, 1.0)
		else:
			self.drive(-.5, 1.0)

if __name__=="__main__":
    rospy.init_node('blob_steering')
    e = blob_steering()
    rospy.spin()
