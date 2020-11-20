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


class image_converter:

    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)
        # initialize a publisher to send images from camera1 to a topic named image_topic1
        self.image_pub1 = rospy.Publisher("image_topic1", Image, queue_size=1)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.callback1)
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

        self.cenBluePublisher = rospy.Publisher("/Image1/Blue", Float64MultiArray, queue_size=10)
        self.cenGreenPublisher = rospy.Publisher("/Image1/Green", Float64MultiArray, queue_size=10)
        self.cenRedPublisher = rospy.Publisher("/Image1/Red", Float64MultiArray, queue_size=10)
        self.cenYellowPublisher = rospy.Publisher("/Image1/Yellow", Float64MultiArray, queue_size=10)
        self.cenBlue = Float64MultiArray()
        self.cenGreen = Float64MultiArray()
        self.cenRed = Float64MultiArray()
        self.cenYellow = Float64MultiArray()

        # Recieve data from camera 1, process it, and publish

    def callback1(self, data):
        # Recieve the image
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Uncomment if you want to save the image
        cv2.imwrite('image_copy.png', self.cv_image1)

        # im1=cv2.imshow('window1', self.cv_image1)
        # cv2.waitKey(1)

        self.CalculateJoints(self.cv_image1)

        # self.robot_joint2_pub.publish(self.joint2)
        # self.robot_joint4_pub.publish(self.joint4)
        # Publish the results
        try:
            # self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
            self.cenBluePublisher.publish(self.cenBlue)
            self.cenGreenPublisher.publish(self.cenGreen)
            self.cenRedPublisher.publish(self.cenRed)
            self.cenYellowPublisher.publish(self.cenYellow)
        except CvBridgeError as e:
            print(e)

        # add code to move joints
        # cur_time = np.array([rospy.get_time()])-self.t0
        # joint2new=Float64()
        # joint2new.data= np.pi/2 * np.sin(cur_time * np.pi/15)
        # print(joint2new)
        # joint3new=Float64()
        # joint3new.data= np.pi/2 * np.sin(cur_time * np.pi/18)
        # joint4new=Float64()
        # joint4new.data= np.pi/2 * np.sin(cur_time * np.pi/20)
        # self.robot_joint2_new_pub.publish(joint2new)
        # self.robot_joint3_new_pub.publish(joint3new)
        # self.robot_joint4_new_pub.publish(joint4new)

    def CalculateJoints(self, Image):

        ret, threshold = cv2.threshold(Image, 100, 255, cv2.THRESH_BINARY)

        dist = self.pixel2meter(Image)
        yellow_joint_position = self.detectYellowCenter(Image)

        self.cenBlue.data = self.transform(self.detectBlueCenter(Image), dist, yellow_joint_position)
        self.cenGreen.data = self.transform(self.detectGreenCenter(Image), dist, yellow_joint_position)
        self.cenRed.data = self.transform(self.detectRedCenter(Image), dist, yellow_joint_position)
        self.cenYellow.data = self.transform(self.detectYellowCenter(Image), dist, yellow_joint_position)
        print(f"camera 1 (y,z): {self.transform(self.detectGreenCenter(Image), dist, yellow_joint_position)}")


        # joint1 = np.arctan2(cenYellow[0] - cenBlue[0],cenYellow[1] - cenBlue[1])
        # joint2 = np.arctan2(cenBlue[0] - cenGreen[0], cenBlue[1] - cenGreen[1]) - joint1
        # joint3 = np.arctan2(cenGreen[0] - cenRed[0], cenGreen[1] - cenRed[1]) - joint1 - joint2

        return "a"

    def transform(self, coord, dist, origin):
        """
        :input: Coordinates in pixels
        :return: Coordinates in metres, with centre at yellow joint.
        """
        reflection = np.array([
            [1, 0],
            [0, -1]
        ])
        return dist * (np.matmul(reflection, coord - origin))

    # Calculate the conversion from pixel to meter
    def pixel2meter(self, image):
        # Obtain the centre of each coloured blob
        circle1Pos = self.detectBlueCenter(image)
        # circle2Pos = self.detectGreenCenter(image)
        circle2Pos = self.detectYellowCenter(image)

        # find the distance between two circles
        dist = np.sum((circle1Pos - circle2Pos) ** 2)
        return 2.5 / np.sqrt(dist)

    def detectRedCenter(self, image):
        lower_red = np.array([0, 0, 100])
        upper_red = np.array([0, 0, 255])
        maskred = cv2.inRange(image, lower_red, upper_red)

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 100, 20), (10, 255, 255))

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        maskOrange = cv2.inRange(hsv, (10, 100, 20), (25, 255, 255))

        cRed = self.detectCenter(mask)

        return cRed

    def detectBlueCenter(self, image):
        # Blue Blob
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        maskblue = cv2.inRange(hsv, (105, 100, 20), (135, 255, 255))
        cBlue = self.detectCenter(maskblue)

        return cBlue

    def detectGreenCenter(self, image):
        # Green Blob
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        maskgreen = cv2.inRange(hsv, (50, 100, 20), (70, 255, 255))
        # print(np.unique(maskgreen))
        # ret, thresh = cv2.threshold(maskgreen, 127, 255, 0)
        # contours, hierarchy = cv2.findContours(thresh, 1, 2)
        # cnt = contours[0]
        cGreen = self.detectCenter(maskgreen)

        return cGreen

    def detectYellowCenter(self, image):
        # Yellow Blob
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        maskyellow = cv2.inRange(hsv, (20, 100, 20), (30, 255, 255))
        cYellow = self.detectCenter(maskyellow)

        return cYellow

    def detectCenter(self, image):

        Mnts = cv2.moments(image)
        cX = int(Mnts["m10"] / Mnts["m00"])
        cY = int(Mnts["m01"] / Mnts["m00"])

        return np.array([cX, cY])


# call the class
def main(args):
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
