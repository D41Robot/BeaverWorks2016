#!/usr/bin/env python
# image is 1280X720
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import threading
import numpy as np
size = 0

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
        lower = np.array([50,100,100])
        upper = np.array([70,255,255])

    # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower, upper)


#Create contours
	contours = cv2.findContours(mask, cv2.cv.CV_RETR_EXTERNAL, cv2.cv.CV_CHAIN_APPROX_NONE)[0]
#Draw contours, nonadjusted
#        cv2.drawContours(image_cv, contours, -1, np.array([12,240,210]))
 
        for i in contours: #Check the size of the contour
              size = cv2.contourArea(i)
              print size
              if size > 1000: #If the contour is larger than 1000 pixrls
                  x,y,w,h = cv2.boundingRect(i) #Draw a box around the contour
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

