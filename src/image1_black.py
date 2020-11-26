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
        self.dist = None
        self.origin = None
        self.yellow_centre = None
        self.blue_centre = None
        self.green_centre = None
        self.red_centre = None
        self.calibrated = False

        self.cenBluePublisher = rospy.Publisher("/Image1/Blue", Float64MultiArray, queue_size=10)
        self.cenGreenPublisher = rospy.Publisher("/Image1/Green", Float64MultiArray, queue_size=10)
        self.cenRedPublisher = rospy.Publisher("/Image1/Red", Float64MultiArray, queue_size=10)
        self.cenYellowPublisher = rospy.Publisher("/Image1/Yellow", Float64MultiArray, queue_size=10)
        self.cenBlue = Float64MultiArray()
        self.cenGreen = Float64MultiArray()
        self.cenRed = Float64MultiArray()
        self.cenYellow = Float64MultiArray()


    def callback1(self, data):
        # Receive the image
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # img = normalize_rgb(self.cv_image1)
        img = self.cv_image1
        # convert to grayscale
        bw = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Threshold to only keep the black links and joints and ignore all other objects in the scene
        _, bw = cv2.threshold(bw, 10, 255, cv2.THRESH_BINARY)
        # Invert the image for use with the distance transform
        bw = 255 - bw
        # Calculate distace from each pixel to the nearest edge
        dist = cv2.distanceTransform(bw, cv2.DIST_L2, 3)
        # Normalize the distance image for range = {0.0, 1.0}
        # so we can visualize and threshold it
        cv2.normalize(dist, dist, 0, 1.0, cv2.NORM_MINMAX)

        # Choose the first joint as the point that is furthest from any edge.
        centre1 = np.unravel_index(dist.argmax(), dist.shape)
        # Remove this joint from the image
        bw[centre1[0] - 10:centre1[0] + 10, centre1[1] - 10:centre1[1] + 10] = 0
        # Choose the second joint as the point that is furthest from an edge now that joint 1 has been removed
        dist = cv2.distanceTransform(bw, cv2.DIST_L2, 3)
        centre2 = np.unravel_index(dist.argmax(), dist.shape)
        # Remove the second joint from the image
        bw[centre2[0] - 10:centre2[0] + 10, centre2[1] - 10:centre2[1] + 10] = 0

        # We need to decide which of centre1 and centre 2 are the blue and yellow joint.
        # Yellow joint should be below the blue joint.
        # This is pretty reliable, although the red joint will sometimes be mistaken for blue
        # This should be correctable when data is amalgamated from both cameras.
        # print(centre1, centre2)

        if not self.calibrated:
            if centre1[0] > centre2[0]:
                self.yellow_centre = np.array([centre1[1], centre1[0]])
                self.blue_centre = np.array([centre2[1], centre2[0]])
            else:
                self.blue_centre = np.array([centre1[1], centre1[0]])
                self.yellow_centre = np.array([centre2[1], centre2[0]])

            # Calibrate the pixels to metres conversion.
            # Only do this on the first iteration, as this is when the robot will be in a knowable
            # state, and the
            self.dist = pixel2meter(self.yellow_centre, self.blue_centre, 2.5)
            self.origin = self.yellow_centre

            self.calibrated = True

        # print(transform(self.yellow_centre, self.dist, self.yellow_centre))
        # print(transform(self.blue_centre, self.dist, self.yellow_centre))


        # Remove this first link:
        link = np.array(centre2) - np.array(centre1)
        for i in range(1, 10):
            coord = np.array(centre1) + np.around(link * i/10).astype(int)
            bw[coord[0] - 12:coord[0] + 12, coord[1] - 12:coord[1] + 12] = 0
            # img = cv2.circle(img, (coord[1], coord[0]), 2, (255, 0, 0), 2)

        # Find the next link, in most cases this will be either the red or green link.
        dist = cv2.distanceTransform(bw, cv2.DIST_L2, 3)
        centre3 = np.unravel_index(dist.argmax(), dist.shape)

        # Remove the third joint from the image
        bw[centre3[0] - 10:centre3[0] + 10, centre3[1] - 10:centre3[1] + 10] = 0
        dist = cv2.distanceTransform(bw, cv2.DIST_L2, 3)
        centre4 = np.unravel_index(dist.argmax(), dist.shape)

        centre3 = np.array([centre3[1], centre3[0]])
        centre4 = np.array([centre4[1], centre4[0]])

        # print(f"centre 3 is {centre3}")
        # print(f"centre 4 is {centre4}")
        # print(self.blue_centre)
        if np.linalg.norm(centre3 - self.blue_centre) < np.linalg.norm(centre4 - self.blue_centre):
            self.green_centre = centre3
            self.red_centre = centre4
        else:
            self.green_centre = centre4
            self.red_centre = centre3


        img = cv2.circle(img, (self.yellow_centre[0], self.yellow_centre[1]), 2, (0, 255, 255), 2)
        img = cv2.circle(img, (self.blue_centre[0], self.blue_centre[1]), 2, (255, 0, 0), 2)

        img = cv2.circle(img, (self.green_centre[0], self.green_centre[1]), 2, (0, 255, 0), 2)
        img = cv2.circle(img, (self.red_centre[0], self.red_centre[1]), 2, (0, 0, 255), 2)


        yellow = transform(self.yellow_centre, self.dist, self.origin)
        blue = transform(self.blue_centre, self.dist, self.origin)
        green = transform(self.green_centre, self.dist, self.origin)
        red = transform(self.red_centre, self.dist, self.origin)

        # This camera records the x-z plane
        # If the x values are all close together there is likely occlusion and the z coordinate cannot be trusted
        if abs(green[0]) < 0.3:
            green = np.array([green[0], 0])
        if abs(red[0]) < 0.3:
            red = np.array([red[0], 0])

        print(green, red)
        # cv2.imshow('window2', bw)
        cv2.imshow('window1', img)
        cv2.waitKey(1)
        try:
            self.cenYellow.data = yellow
            self.cenBlue.data = blue
            self.cenGreen.data = green
            self.cenRed.data = red
            self.cenYellowPublisher.publish(self.cenYellow)
            self.cenBluePublisher.publish(self.cenBlue)
            self.cenGreenPublisher.publish(self.cenGreen)
            self.cenRedPublisher.publish(self.cenRed)
        except CvBridgeError as e:
            print(e)



def pixel2meter(coord1, coord2, true_distance):
    # find the distance between two circles
    dist = np.sum((coord1 - coord2) ** 2)
    return true_distance / np.sqrt(dist)


def transform(coord, dist, origin):
    """
    :input: Coordinates in pixels
    :return: Coordinates in metres, with centre at yellow joint.
    """
    reflection = np.array([
        [1, 0],
        [0, -1]
    ])
    return dist * (np.matmul(reflection, coord - origin))


def normalize_rgb(image):
    image = image.astype(np.uint16)
    channel_sum = image[:, :, 0] + image[:, :, 1] + image[:, :, 2]
    # Set any zero pixel values to q to avoid division by zero errors
    channel_sum = np.repeat(channel_sum[:, :, np.newaxis], 3, axis=2)
    normalised_img = 255 * np.divide(image, channel_sum)
    normalised_img = np.around(normalised_img).astype(np.uint8)
    return normalised_img


def transform(coord, dist, origin):
    """
    :input: Coordinates in pixels
    :return: Coordinates in metres, with centre at yellow joint.
    """
    reflection = np.array([
        [1, 0],
        [0, -1]
    ])
    return dist * (np.matmul(reflection, coord - origin))


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
