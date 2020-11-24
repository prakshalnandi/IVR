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
        self.image1SphereSub = rospy.Subscriber("/Image1/Sphere", Float64MultiArray, self.sphere_yz_callback)
        self.image2SphereSub = rospy.Subscriber("/Image2/Sphere", Float64MultiArray, self.sphere_xz_callback)

        self.image1Blue = Float64MultiArray()
        self.image1Red = Float64MultiArray()
        self.image1Green = Float64MultiArray()
        self.image1Yellow = Float64MultiArray()
        self.image2Blue = Float64MultiArray()
        self.image2Red = Float64MultiArray()
        self.image2Green = Float64MultiArray()
        self.image2Yellow = Float64MultiArray()
        self.sphere_yz = Float64MultiArray()
        self.sphere_xz = Float64MultiArray()

        self.robot_joint2_pub = rospy.Publisher("/Image/Joint2", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/Image/Joint3", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/Image/Joint4", Float64, queue_size=10)
        self.joint2 = Float64()
        self.joint3 = Float64()
        self.joint4 = Float64()
        self.target_x_pub = rospy.Publisher("/target/x_position", Float64, queue_size=10)
        self.target_y_pub = rospy.Publisher("/target/y_position", Float64, queue_size=10)
        self.target_z_pub = rospy.Publisher("/target/z_position", Float64, queue_size=10)
        self.target_x = Float64()
        self.target_y = Float64()
        self.target_z = Float64()
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

    def sphere_yz_callback(self, data):
        self.sphere_yz.data = data.data

    def sphere_xz_callback(self, data):
        self.sphere_xz.data = data.data

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
            # print(self.sphere_yz.data, self.sphere_xz.data)
            sphere_position = np.array([self.sphere_xz.data[0], self.sphere_yz.data[0],
                                        (self.sphere_xz.data[1] + self.sphere_yz.data[1]) / 2])
            print(f"Sphere is at: {sphere_position}")
            self.target_x = sphere_position[0]
            self.target_y = sphere_position[1]
            self.target_z = sphere_position[2]
            self.target_x_pub.publish(self.target_x)
            self.target_y_pub.publish(self.target_y)
            self.target_z_pub.publish(self.target_z)

            # To get the angle for joint 4 we have to use the forward kinematics matrix.
            # print(f"camera1: {self.image1Red.data}, camera 2: {self.image2Red.data}")
            red_pos = np.array([self.image2Red.data[0], self.image1Red.data[0], (self.image1Red.data[1] +
                                                                                 self.image2Red.data[1]) / 2])
            green_pos = np.array([self.image2Green.data[0], self.image1Green.data[0],
                                  max(2.5, (self.image1Green.data[1] + self.image2Green.data[1]) / 2)])
            blue_pos = np.array([self.image2Blue.data[0], self.image1Blue.data[0], (self.image1Blue.data[1] +
                                                                                    self.image2Blue.data[1]) / 2])
            print("-" * 20)

            link1 = blue_pos  # yellow joint is always at (0,0,0)
            link2 = green_pos - blue_pos
            link3 = red_pos - green_pos

            ############### Testing ##################
            print(f"green_pos {green_pos}")
            link2_norm = link2 / np.linalg.norm(link2)
            print("link2 norm: ", link2_norm)
            # happy with this estimation
            theta3 = np.arcsin(link2_norm[0])
            self.joint3.data = theta3
            # theta2 = np.arccos(link2_norm[2] / np.cos(theta3))
            # if abs(np.cos(theta3)) >= 0.1:
            theta2 = np.arccos(link2_norm[2] / np.cos(theta3))
            t2_plus = theta2
            t2_minus = - theta2


            # use FK matrix to find out where theta should be positive or negative
            # Fix theta1 to be 0 (which needs to be -pi/2 in the FK matrix)

            if abs(abs(theta3) - np.pi/2) >= 0.2:
                # if np.linalg.norm(link2 - link2_plus) < np.linalg.norm(link2 - link2_minus):
                #     self.joint2.data = theta2
                # else:
                #     self.joint2.data = -theta2
                print(f"link2 is: {link2}")
                if link2[1] < 0:
                    self.joint2.data = theta2
                else:
                    self.joint2.data = -theta2
                print(f"Theta2 estimate is: {self.joint2.data}")
            print(f"Theta3 estimate is: {self.joint3.data}")


            # need to subtract by pi/2 for theta1 and theta2 as this is what is done in the FK derivation
            theta1 = -np.pi / 2
            theta2 = self.joint2.data - np.pi / 2
            theta3 = self.joint3.data

            # define some values that will simplify equations further below
            c1 = np.cos(theta1)
            c2 = np.cos(theta2)
            c3 = np.cos(theta3)
            s1 = np.sin(theta1)
            s2 = np.sin(theta2)
            s3 = np.sin(theta3)

            # Find the angle between link2 and link3 using the dot product and arccos
            theta4 = np.arccos(link2.dot(link3) / (np.linalg.norm(link3) * np.linalg.norm(link2)))
            s4 = np.sin(theta4)
            c4 = np.cos(theta4)
            t2 = np.arccos(link3[2] / (-3 * np.cos(theta4)))
            t2_minus = np.arccos(link3[2] / (-3 * np.cos(-theta4)))
            # print(f"Theta2 guess: {t2, t2_minus}")
            # Arccos always gives a positive angle, so we need to test if we should use that
            # or if the negative is in fact correct.
            # To test we use the predicted value from the FK derivation.
            # That is link3 = A_04 - A_03 (only looking at the translation parts of the matrices.
            link3_plus = np.array([
                -3 * c1 * s2 * s4 + 3 * c4 * (c1 * c2 * c3 - s1 * s3),
                3 * c4 * (c1 * s3 + c2 * c3 * s1) - 3 * s1 * s2 * s4,
                -3 * c2 * s4 - 3 * c3 * c4 * s2,
            ])
            s4 = np.sin(-theta4)
            c4 = np.cos(-theta4)
            link3_minus = np.array([
                -3 * c1 * s2 * s4 + 3 * c4 * (c1 * c2 * c3 - s1 * s3),
                3 * c4 * (c1 * s3 + c2 * c3 * s1) - 3 * s1 * s2 * s4,
                -3 * c2 * s4 - 3 * c3 * c4 * s2,
            ])
            # If the prediction using negative theta is closer to the measured
            # value from CV then we use that one. Otherwise use positive value.
            # fails if theta1 ~> 3
            if np.linalg.norm(link3 - link3_minus) < np.linalg.norm(link3 - link3_plus):
                self.joint4.data = -theta4
            else:
                self.joint4.data = theta4

            # print("theta4: ", self.joint4.data)
            self.robot_joint2_pub.publish(self.joint2)
            if abs(self.joint3.data**2 - (np.pi/2)**2) >= 0.3:
                # not the case keep last known value
                self.robot_joint3_pub.publish(self.joint3)
            self.robot_joint4_pub.publish(self.joint4)

        except CvBridgeError as e:
            print(e)

    # Uncomment if you want to save the image
    # print("publishing")
    # cv2.imwrite('image_copy.png', self.cv_image1)

    # im1=cv2.imshow('window1', self.cv_image1)
    # cv2.waitKey(1)


def fk3(angles):
    theta1 = angles[0] - np.pi / 2
    theta2 = angles[1] - np.pi / 2
    theta3 = angles[2]
    theta4 = angles[3]

    c1 = np.cos(theta1)
    c2 = np.cos(theta2)
    c3 = np.cos(theta3)
    c4 = np.cos(theta4)
    s1 = np.sin(theta1)
    s2 = np.sin(theta2)
    s3 = np.sin(theta3)
    s4 = np.sin(theta4)
    k = np.array([
        [-c1 * s2 * s4 + c4 * (c1 * c2 * c3 - s1 * s3), -c1 * c4 * s2 - s4 * (c1 * c2 * c3 - s1 * s3),
         -c1 * c2 * s3 - c3 * s1,
         3.5 * c1 * c2 * c3 - 3 * c1 * s2 * s4 + 3 * c4 * (c1 * c2 * c3 - s1 * s3) - 3.5 * s1 * s3],
        [c4 * (c1 * s3 + c2 * c3 * s1) - s1 * s2 * s4, -c4 * s1 * s2 - s4 * (c1 * s3 + c2 * c3 * s1),
         c1 * c3 - c2 * s1 * s3,
         3.5 * c1 * s3 + 3.5 * c2 * c3 * s1 + 3 * c4 * (c1 * s3 + c2 * c3 * s1) - 3 * s1 * s2 * s4],
        [-c2 * s4 - c3 * c4 * s2, -c2 * c4 + c3 * s2 * s4, s2 * s3,
         -3 * c2 * s4 - 3 * c3 * c4 * s2 - 3.5 * c3 * s2 + 2.5],
        [0, 0, 0, 1]
    ])
    return k[:, -1][:-1]


if __name__ == '__main__':
    subscribe()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
