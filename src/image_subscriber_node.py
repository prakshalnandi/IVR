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
        rospy.init_node('subscriber_node', anonymous=True)
        # self.subscriber_name = rospy.Subscriber("/Data", Float64, self.callback)
        # initialize a subscriber to recieve messages rom a topic named image_topic1 and use callback function to recieve data
        # self.image1 = rospy.Subscriber("image_topic1",Image,self.callback1)

        self.image1BlueSub = rospy.Subscriber("/Image1/Blue", Float64MultiArray, self.callback1)
        self.image1RedSub = rospy.Subscriber("/Image1/Red", Float64MultiArray, self.callback2)
        self.image1GreenSub = rospy.Subscriber("/Image1/Green", Float64MultiArray, self.callback3)
        self.image1YellowSub = rospy.Subscriber("/Image1/Yellow", Float64MultiArray, self.callback4)
        self.image2BlueSub = rospy.Subscriber("/Image2/Blue", Float64MultiArray, self.callback5)
        self.image2RedSub = rospy.Subscriber("/Image2/Red", Float64MultiArray, self.callback6)
        self.image2GreenSub = rospy.Subscriber("/Image2/Green", Float64MultiArray, self.callback7)
        self.image2YellowSub = rospy.Subscriber("/Image2/Yellow", Float64MultiArray, self.callback8)

        self.image1Blue = Float64MultiArray()
        self.image1Red = Float64MultiArray()
        self.image1Green = Float64MultiArray()
        self.image1Yellow = Float64MultiArray()
        self.image2Blue = Float64MultiArray()
        self.image2Red = Float64MultiArray()
        self.image2Green = Float64MultiArray()
        self.image2Yellow = Float64MultiArray()

        self.robot_joint2_pub = rospy.Publisher("/Image/Joint2", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/Image/Joint3", Float64, queue_size=10)
        self.joint2 = Float64()
        self.joint3 = Float64()
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

    def callback1(self, data):
        self.image1Blue.data = data.data

    def callback2(self, data):
        self.image1Red.data = data.data

    def callback3(self, data):
        self.image1Green.data = data.data

    def callback4(self, data):
        self.image1Yellow.data = data.data

    def callback5(self, data):
        self.image2Blue.data = data.data

    def callback6(self, data):
        self.image2Red.data = data.data

    def callback7(self, data):
        self.image2Green.data = data.data

    def callback8(self, data):
        self.image2Yellow.data = data.data
        # print(self.image2Yellow.data)
        # print(self.image1Yellow.data)
        # print(self.image1Green.data)
        # print(self.image1Red.data)
        # print(self.image1Blue.data)
        # print(self.image2Green.data)
        # print(self.image2Red.data)
        # print(self.image2Blue.data)

        # self.joint3.data = np.arctan2(self.image2Yellow.data[0] - self.image2Blue.data[0],self.image2Yellow.data[1] - self.image2Blue.data[1])

        try:
            self.joint2.data = -np.arctan2(self.image1Green.data[0] - self.image1Blue.data[0],
                                          self.image1Green.data[1] - self.image1Blue.data[1])
            self.joint3.data = -np.arctan2(self.image2Green.data[0] - self.image2Blue.data[0],
                                           self.image2Green.data[1] - self.image2Blue.data[1])
            # self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.robot_joint2_pub.publish(self.joint2)
            self.robot_joint3_pub.publish(self.joint3)
        except CvBridgeError as e:
            print(e)

    # Uncomment if you want to save the image
    # print("publishing")
    # cv2.imwrite('image_copy.png', self.cv_image1)

    # im1=cv2.imshow('window1', self.cv_image1)
    # cv2.waitKey(1)


if __name__ == '__main__':
    subscribe()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
