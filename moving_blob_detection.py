#!/usr/bin/env python
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import threading
import numpy as np
contour_size = 1500

class blob_detector:
    def __init__(self):
        self.node_name = "blob_detector"
        self.thread_lock = threading.Lock()
        self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color",\
                Image, self.cbImage, queue_size=1)
        self.pub_image = rospy.Publisher("~blob_detection",\
                Image, queue_size=1)
        self.bridge = CvBridge()

        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def cbImage(self,image_msg):
        thread = threading.Thread(target=self.processImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()


    def processImage(self, image_msg):
        if not self.thread_lock.acquire(False):
            return
        image_cv = self.bridge.imgmsg_to_cv2(image_msg)

    # Convert BGR to HSV
        hsv = cv2.cvtColor(image_cv, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
        lower = np.array([0,100,200])
        upper = np.array([15,255,255])

    # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower, upper)

    # Bitwise-AND mask and original image
        #res = cv2.bitwise_and(cap,cap, mask= mask)


#Create contours
	contours = cv2.findContours(mask, cv2.cv.CV_RETR_EXTERNAL, cv2.cv.CV_CHAIN_APPROX_NONE)[0]
#Draw contours
#        cv2.drawContours(image_cv, contours, -1, np.array([12,240,210]))
 
        for i in contours:
              size = cv2.contourArea(i)
              if size > 1000:
                  x,y,w,h = cv2.boundingRect(i)
                  cv2.rectangle(image_cv, (x,y) , (x+w, y+h), (0,255,0),2)
            

        
        try:
            self.pub_image.publish(\
                    self.bridge.cv2_to_imgmsg(image_cv, "bgr8"))
        except CvBridgeError as e:
            print(e)
        self.thread_lock.release()


if __name__=="__main__":
    rospy.init_node('blob_detector')
    e = blob_detector()
    rospy.spin()

