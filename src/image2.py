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
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    
    self.robot_joint3_pub = rospy.Publisher("cal_joint3", Float64, queue_size=10)
    self.joint3 = Float64()
    #print("Added Error")
	

  # Recieve data, process it, and publish
  def callback2(self,data):
    # Recieve the image
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    # Uncomment if you want to save the image
    #print("Writing File Now")
    blnTrue = cv2.imwrite('image_copy2.png', self.cv_image2)
    print(blnTrue)
    #im2=cv2.imshow('window2', self.cv_image2)
    cv2.waitKey(1)
    
    self.CalculateJoints(self.cv_image2)
    self.robot_joint3_pub.publish(self.joint3)
    
    #### Publish the results
    ###try: 
      ###self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
    ###except CvBridgeError as e:
      ###print(e)
   
    
  def CalculateJoints(self,Image):
  
    ret, threshold = cv2.threshold(Image, 100, 255, cv2.THRESH_BINARY)

    dist = self.pixel2meter(threshold)

    cenBlue = dist * self.detectBlueCenter(threshold)
    #cenRed = dist * self.detectRedCenter(Image)
    cenGreen = dist * self.detectGreenCenter(threshold)
    #cenYellow = dist * self.detectYellowCenter(Image)

    #joint1 = np.arctan2(cenYellow[0] - cenBlue[0], cenYellow[1] - cenBlue[1])
    self.joint3 = -np.arctan2(cenBlue[0] - cenGreen[0], cenBlue[1] - cenGreen[1])
        
    print(str(self.joint3))
       
    
    return "a"


  # Calculate the conversion from pixel to meter
  def pixel2meter(self,image):
    # Obtain the centre of each coloured blob
    circle1Pos = self.detectBlueCenter(image)
    circle2Pos = self.detectGreenCenter(image)

    # find the distance between two circles
    dist = np.sum((circle1Pos - circle2Pos) ** 2)
    return 3.5 / np.sqrt(dist)

  def detectRedCenter(self,image):
  
    lower_red = np.array([0, 0, 100])
    upper_red = np.array([0, 0, 255])
    maskred = cv2.inRange(image, lower_red, upper_red)

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv,  (0, 100, 20), (10, 255, 255))

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    maskOrange = cv2.inRange(hsv, (10, 100, 20), (25, 255, 255))

    cRed = self.detectCenter(maskred)
    return cRed
    # MRed = cv2.moments(maskred)


  def detectBlueCenter(self,image):
  
    lower_blue = np.array([100, 0, 0])
    upper_blue = np.array([255, 0, 0])
    maskblue = cv2.inRange(image, lower_blue, upper_blue)
    cBlue = self.detectCenter(maskblue)
    
    return cBlue


  def detectGreenCenter(self,image):
  
    # Green Blob
    lower_green = np.array([0, 100, 0])
    upper_green = np.array([0, 255, 0])
    maskgreen = cv2.inRange(image, lower_green, upper_green)
    cGreen = self.detectCenter(maskgreen)
    
    return cGreen

  def detectYellowCenter(self,image):
    # Yellow Blob
    lower_yellow = np.array([0, 200, 200])
    upper_yellow = np.array([0, 255, 255])
    maskyellow = cv2.inRange(image, lower_yellow, upper_yellow)
    cYellow = self.detectCenter(maskyellow)

    #hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    #mask = cv2.inRange(hsv, (10, 100, 20), (25, 255, 255))


    return cYellow

  def detectCenter(self,image):
    MRed = cv2.moments(image)
    #print(type(MRed))
    if (MRed["m00"] != 0):
    	cRedX = int(MRed["m10"] / MRed["m00"])
    	cRedY = int(MRed["m01"] / MRed["m00"])

    #print(cRedX, cRedY)
    return np.array([cRedX,cRedY])

    # Publish the results
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
    except CvBridgeError as e:
      print(e)

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


