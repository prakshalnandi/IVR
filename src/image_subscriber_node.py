#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError

class subscribe:

    def __init__(self):
        rospy.init_node('subscriber_node', anonymous = True)
        #self.subscriber_name = rospy.Subscriber("/Data", Float64, self.callback)
        # initialize a subscriber to recieve messages rom a topic named image_topic1 and use callback function to recieve data
        self.image1 = rospy.Subscriber("image_topic1",Image,self.callback1)
	# initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

    def callback1(self, data):
         # Recieve the image
        try:
          self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
          
        except CvBridgeError as e:
          print(e)
        
        # Uncomment if you want to save the image
        print("publishing")
        cv2.imwrite('image_copy.png', self.cv_image1)

        #im1=cv2.imshow('window1', self.cv_image1)
        cv2.waitKey(1)


if __name__ == '__main__':
    subscribe()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")




