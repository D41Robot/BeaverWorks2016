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
		self.stopped=False
		rospy.Subscriber("image_size", Float32, self.stopper)
		rospy.Subscriber("imagepos_error", Float32, self.calcdrive)

		self.pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=5)

	def drive(self, angle, speed):

        	ackmsg = AckermannDriveStamped()
       		ackmsg.drive.speed = speed
        	ackmsg.drive.steering_angle = angle

        	self.pub.publish(ackmsg)
		#print "published"
	
	def stopper(self, msg):
		self.stopped=msg.data>500

	def calcdrive(self, msg):
                msg = 640.0 - msg.data
		#float f = float
		#print str(type(msg))
                print msg
		if self.stopped:
			self.drive(0,0)
                        return
		elif msg>0:
			self.drive(.5, 0.5)
		else:
			self.drive(-.5, 0.5)

if __name__=="__main__":
    rospy.init_node('blob_steering')
    e = blob_steering()
    rospy.spin()
