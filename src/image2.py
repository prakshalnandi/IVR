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
        # initialize a publisher to send images from camera2 to a topic named image_topic2
        self.image_pub2 = rospy.Publisher("image_topic2", Image, queue_size=1)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.callback2)
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

        self.cenBluePublisher = rospy.Publisher("/Image2/Blue", Float64MultiArray, queue_size=10)
        self.cenGreenPublisher = rospy.Publisher("/Image2/Green", Float64MultiArray, queue_size=10)
        self.cenRedPublisher = rospy.Publisher("/Image2/Red", Float64MultiArray, queue_size=10)
        self.cenYellowPublisher = rospy.Publisher("/Image2/Yellow", Float64MultiArray, queue_size=10)
        self.spherePublisher = rospy.Publisher("/Image2/Sphere", Float64MultiArray, queue_size=10)
        self.cenBlue = Float64MultiArray()
        self.cenGreen = Float64MultiArray()
        self.cenRed = Float64MultiArray()
        self.cenYellow = Float64MultiArray()
        self.sphere = Float64MultiArray()

    # Recieve data, process it, and publish
    def callback2(self, data):
        # Recieve the image
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        # Uncomment if you want to save the image

        # blnTrue = cv2.imwrite('image_copy2.png', self.cv_image2)


        self.CalculateJoints(self.cv_image2)

        # Publish the results
        try:
            # self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
            self.cenBluePublisher.publish(self.cenBlue)
            self.cenGreenPublisher.publish(self.cenGreen)
            self.cenRedPublisher.publish(self.cenRed)
            self.cenYellowPublisher.publish(self.cenYellow)
            self.spherePublisher.publish(self.sphere)
        except CvBridgeError as e:
            print(e)
        img = self.cv_image2
        # print(self.cenRed)
        lower_red = np.array([0, 0, 100])
        upper_red = np.array([0, 0, 255])
        maskred = cv2.inRange(img, lower_red, upper_red)

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 100, 20), (10, 255, 255))

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        maskOrange = cv2.inRange(hsv, (10, 100, 20), (25, 255, 255))
        data = tuple(self.detectCenter(mask))

        img = cv2.circle(img, data, 1, (0, 255, 0), thickness=1, lineType=8, shift=0)
        # cv2.imshow('window2', img)
        # cv2.waitKey(1)


    def CalculateJoints(self, Image):

        ret, threshold = cv2.threshold(Image, 100, 255, cv2.THRESH_BINARY)

        dist = self.pixel2meter(Image)
        yellow_joint_position = self.detectYellowCenter(Image)
        blue_joint_position = self.detectBlueCenter(Image)
        green_joint_position = self.detectGreenCenter(Image)
        red_joint_position = self.detectRedCenter(Image)
        sphere = get_target(Image)

        if sphere is not None:
            self.sphere.data = self.transform(sphere, dist, yellow_joint_position)
        if yellow_joint_position is not None:
            self.cenYellow.data = self.transform(yellow_joint_position, dist, yellow_joint_position)
        if blue_joint_position is not None:
            self.cenBlue.data = self.transform(blue_joint_position, dist, yellow_joint_position)
        if green_joint_position is not None:
            self.cenGreen.data = self.transform(green_joint_position, dist, yellow_joint_position)
        if red_joint_position is not None:
            self.cenRed.data = self.transform(red_joint_position, dist, yellow_joint_position)

        # print(f"sphere: {self.sphere.data}")

    def transform(self, coord, dist, origin):
        """
        :input: Coordinates in pixels
        :return: Coordinates in metres, with centre at yellow joint.
        """
        reflection_in_y = np.array([
            [1, 0],
            [0, -1]
        ])
        # Camera 2 faces the back of the xz-plane so we also want to mirror the image.
        # reflection_in_z = np.array([
        #     [-1, 0],
        #     [0, 1]
        # ])
        # return dist * np.matmul(reflection_in_z, (np.matmul(reflection_in_y, coord - origin)))
        return dist * np.matmul(reflection_in_y, coord - origin)

    # Calculate the conversion from pixel to meter
    def pixel2meter(self, image):
        # Obtain the centre of each coloured blob
        circle1Pos = self.detectBlueCenter(image)
        circle2Pos = self.detectYellowCenter(image)

        # find the distance between two circles
        dist = np.sum((circle1Pos - circle2Pos) ** 2)
        return 2.5 / np.sqrt(dist)

    def detectRedCenter(self, image):
        try:
            lower_red = np.array([0, 0, 100])
            upper_red = np.array([0, 0, 255])
            maskred = cv2.inRange(image, lower_red, upper_red)

            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, (0, 100, 20), (10, 255, 255))

            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            maskOrange = cv2.inRange(hsv, (10, 100, 20), (25, 255, 255))
            cRed = self.detectCenter(mask)

            return cRed
        except:
            # Catch all errors, as these are likely due to occlusion
            return None

    def detectBlueCenter(self, image):
        try:
            # Blue Blob
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            maskblue = cv2.inRange(hsv, (105, 100, 20), (135, 255, 255))
            cBlue = self.detectCenter(maskblue)

            return cBlue
        except:
            # Catch all errors, as these are likely due to occlusion
            return None

    def detectGreenCenter(self, image):
        try:
            # Green Blob
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            maskgreen = cv2.inRange(hsv, (50, 100, 20), (70, 255, 255))
            cGreen = self.detectCenter(maskgreen)

            return cGreen
        except:
            # Catch all errors, as these are likely due to occlusion
            return None

    def detectYellowCenter(self, image):
        try:
            # Yellow Blob
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            maskyellow = cv2.inRange(hsv, (20, 100, 20), (30, 255, 255))
            cYellow = self.detectCenter(maskyellow)

            return cYellow
        except:
            # Catch all errors, as these are likely due to occlusion
            return None

    def detectCenter(self, image):

        try:
            Mnts = cv2.moments(image)
            cX = int(Mnts["m10"] / Mnts["m00"])
            cY = int(Mnts["m01"] / Mnts["m00"])

            return np.array([cX, cY])

        except:
            pass
        ## Publish the results
        # try:
        #  self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
        # except CvBridgeError as e:
        #  print(e)

def get_target(image):
    # RGB-Normalise the image to remove lighting effects.
    # This will make it easier to threshold the ball and cuboid away
    # from the rest of the scene
    img = normalize_rgb(image)
    orange_lower = (0, 0, 130)
    orange_upper = (100, 100, 190)
    mask = cv2.inRange(img, orange_lower, orange_upper)
    output = cv2.bitwise_and(image, image, mask=mask)
    output = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
    output = cv2.bitwise_not(output)
    kernel = np.ones((5, 5), np.uint8)
    # dilating removes the specularites
    # a median blur convolution will also do this
    output = cv2.dilate(output, kernel, iterations=1)
    output = cv2.erode(output, kernel, iterations=2)

    ret, threshold = cv2.threshold(output, 200, 255, 0)
    contours, hierarchy = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    sphere, cuboid = None, None
    try:
        for contour in contours[1:]:
            moments = cv2.moments(contour)
            centre_x = int(moments["m10"] / moments["m00"])
            centre_y = int(moments["m01"] / moments["m00"])

            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)

            compactness = 2 * np.sqrt(area * np.pi) / perimeter
            hull = cv2.convexHull(contour)
            hull_area = cv2.contourArea(hull)
            hull_perimeter = cv2.arcLength(hull, True)
            convexity = hull_perimeter / perimeter
            # print(f"convexity, compactness: {convexity, compactness}")
            # The sphere consistently shows compactness around 0.95 and
            # the cuboid around 0.89
            if compactness > 0.925 and convexity > 0.95:
                sphere = np.array([centre_x, centre_y])
            elif compactness <= 0.925 and convexity > 0.95:
                cuboid = np.array([centre_x, centre_y])

        return sphere
    except:
        return None

def normalize_rgb(image):
    image = image.astype(np.uint16)
    channel_sum = image[:, :, 0] + image[:, :, 1] + image[:, :, 2]
    # Set any zero pixel values to q to avoid division by zero errors
    channel_sum = np.repeat(channel_sum[:, :, np.newaxis], 3, axis=2)
    normalised_img = 255 * np.divide(image, channel_sum)
    normalised_img = np.around(normalised_img).astype(np.uint8)
    return normalised_img


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
