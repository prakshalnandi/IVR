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
        self.robot_joints_pub = rospy.Publisher("/Image/Joints", Float64MultiArray, queue_size=10)
        self.joint2 = Float64()
        self.joint3 = Float64()
        self.joint4 = Float64()
        self.joints = Float64MultiArray()
        self.target_x_pub = rospy.Publisher("/target/x_position", Float64, queue_size=10)
        self.target_y_pub = rospy.Publisher("/target/y_position", Float64, queue_size=10)
        self.target_z_pub = rospy.Publisher("/target/z_position", Float64, queue_size=10)
        self.target_pub = rospy.Publisher("/Image/target", Float64MultiArray, queue_size=10)
        self.target_x = Float64()
        self.target_y = Float64()
        self.target_z = Float64()
        self.target = Float64MultiArray()
        self.end_pub = rospy.Publisher("/Image/endeffector", Float64MultiArray, queue_size=10)
        self.end_x = Float64()
        self.end_y = Float64()
        self.end_z = Float64()
        self.end = Float64MultiArray()
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

        try:
            # Add 0.8 metres to the z position to account for the base platform that the robot
            # sits on.
            if self.sphere_xz.data:
                sphere_position = np.array([self.sphere_xz.data[0], self.sphere_yz.data[0], 0.8 +
                                            (self.sphere_xz.data[1] + self.sphere_yz.data[1]) / 2])
                self.target_x = sphere_position[0]
                self.target_y = sphere_position[1]
                self.target_z = sphere_position[2]
                self.target.data = (self.target_x, self.target_y, self.target_z)
                self.target_x_pub.publish(self.target_x)
                self.target_y_pub.publish(self.target_y)
                self.target_z_pub.publish(self.target_z)
                self.target_pub.publish(self.target)
            # To get the angle for joint 4 we have to use the forward kinematics matrix.
            # print(f"camera1: {self.image1Red.data}, camera 2: {self.image2Red.data}")
            red_pos = np.array([self.image2Red.data[0], self.image1Red.data[0], (self.image1Red.data[1] +
                                                                                 self.image2Red.data[1]) / 2])
            green_pos = np.array([self.image2Green.data[0], self.image1Green.data[0],
                                  max(2.5, (self.image1Green.data[1] + self.image2Green.data[1]) / 2)])
            blue_pos = np.array([self.image2Blue.data[0], self.image1Blue.data[0], (self.image1Blue.data[1] +
                                                                                    self.image2Blue.data[1]) / 2])
            print("-" * 20)

            self.end_x = red_pos[0]
            self.end_y = red_pos[1]
            self.end_z = red_pos[2]
            self.end.data = (self.end_x,self.end_y,self.end_z)
            #self.target_x_pub.publish(self.target_x)
            #self.target_y_pub.publish(self.target_y)
            #self.target_z_pub.publish(self.target_z)
            self.end_pub.publish(self.end)

            link1 = blue_pos  # yellow joint is always at (0,0,0)
            link2 = green_pos - blue_pos
            link3 = red_pos - green_pos

            ############### Testing ##################
            link2_norm = link2 / np.linalg.norm(link2)
            theta3 = np.arcsin(link2_norm[0])
            self.joint3.data = theta3
            theta4 = np.arccos(link2.dot(link3) / (np.linalg.norm(link2) * np.linalg.norm(link3)))

            c3 = np.cos(theta3)
            s3 = np.sin(theta3)

            if abs(abs(theta3) - (np.pi/2)) > 0.1:  # i.e.theta3 = pi/2
                # safe to divide by np.cos(theta3)
                theta2 = np.arcsin(link2[2]/(-3.5*np.cos(theta3))) + np.pi/2
            else:
                # Try to get angle using vector for link2 and the z axis
                theta2 = np.arccos(link2.dot(np.array([0, 0, 1])) / (np.linalg.norm(link2)))

            c2 = np.cos(theta2-np.pi/2)
            s2 = np.sin(theta2-np.pi/2)
            link2_plus = np.array([
                3.5 * s3,
                -3.5 * c2*c3,
                -3.5 * c3*s2
            ])
            c2 = np.cos(-theta2-np.pi/2)
            s2 = np.sin(-theta2-np.pi/2)
            link2_minus = np.array([
                3.5 * s3,
                -3.5 * c2*c3,
                -3.5 * c3*s2
            ])
            if np.linalg.norm(link2_plus - link2) < np.linalg.norm(link2_minus - link2):
                self.joint2.data = theta2
            else:
                self.joint2.data = -theta2

            c2 = np.cos(self.joint2.data-np.pi/2)
            s2 = np.sin(self.joint2.data-np.pi/2)
            c4 = np.cos(theta4)
            s4 = np.sin(theta4)
            # Need to verify the sign of theta 4
            # assume theta1 = 0
            link3_plus = np.array([
                3*c4*s3,
                -3*c4*c2*c3,
                -3*c2*s4 - 3*c3*c4*s2
            ])
            c4 = np.cos(-theta4)
            s4 = np.sin(-theta4)
            link3_minus = np.array([
                3*c4*s3,
                -3*c4*c2*c3,
                -3*c2*s4 - 3*c3*c4*s2
            ])
            print("plus2: ", link3_plus)
            print("minus2: ", link3_minus)
            print(link3)
            if np.linalg.norm(link3_plus - link3) < np.linalg.norm(link3_minus - link3):
                self.joint4.data = theta4
            else:
                self.joint4.data = -theta4
                
            self.joints.data = (self.joint2.data,self.joint3.data,self.joint4.data)
            print("t2: ", self.joint2.data)
            print("t3: ", self.joint3.data)
            print("t4: ", self.joint4.data)
            
            self.robot_joint2_pub.publish(self.joint2.data)
            self.robot_joint3_pub.publish(self.joint3.data)
            self.robot_joint4_pub.publish(self.joint4.data)
            self.robot_joints_pub.publish(self.joints)


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
