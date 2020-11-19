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
    
    self.cenBluePublisher = rospy.Publisher("/Image2/Blue", Float64MultiArray, queue_size=10)
    self.cenGreenPublisher = rospy.Publisher("/Image2/Green", Float64MultiArray, queue_size=10)
    self.cenRedPublisher = rospy.Publisher("/Image2/Red", Float64MultiArray, queue_size=10)
    self.cenYellowPublisher = rospy.Publisher("/Image2/Yellow", Float64MultiArray, queue_size=10)
    self.cenBlue = Float64MultiArray()
    self.cenGreen = Float64MultiArray()
    self.cenRed = Float64MultiArray()
    self.cenYellow = Float64MultiArray()
	

  # Recieve data, process it, and publish
  def callback2(self,data):
    # Recieve the image
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    # Uncomment if you want to save the image
    
    #blnTrue = cv2.imwrite('image_copy2.png', self.cv_image2)
    
    #im2=cv2.imshow('window2', self.cv_image2)
    #cv2.waitKey(1)
    
    self.CalculateJoints(self.cv_image2)         
 
    # Publish the results
    try: 
      #self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
      self.cenBluePublisher.publish(self.cenBlue)
      self.cenGreenPublisher.publish(self.cenGreen)
      self.cenRedPublisher.publish(self.cenRed)
      self.cenYellowPublisher.publish(self.cenYellow)
    except CvBridgeError as e:
      print(e)

      
  def CalculateJoints(self,Image):
  
    ret, threshold = cv2.threshold(Image, 100, 255, cv2.THRESH_BINARY)

    dist = self.pixel2meter(Image)

    self.cenBlue.data = dist * self.detectBlueCenter(Image)
    self.cenGreen.data = dist * self.detectGreenCenter(Image)
    self.cenRed.data = dist * self.detectRedCenter(Image)    
    self.cenYellow.data = dist * self.detectYellowCenter(Image)     
    
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
    cRed = self.detectCenter(mask)
    
    return cRed


  def detectBlueCenter(self,image):
    # Blue Blob
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    maskblue = cv2.inRange(hsv, (105, 100, 20), (135, 255, 255))
    cBlue = self.detectCenter(maskblue)
    
    return cBlue

  def detectGreenCenter(self,image):
    # Green Blob
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    maskgreen = cv2.inRange(hsv, (50, 100, 20), (70, 255, 255))
    cGreen = self.detectCenter(maskgreen)
    
    return cGreen

  def detectYellowCenter(self,image):
    # Yellow Blob    
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    maskyellow = cv2.inRange(hsv, (20, 100, 20), (30, 255, 255))
    cYellow = self.detectCenter(maskyellow)
    
    return cYellow

  def detectCenter(self,image):
  
    Mnts = cv2.moments(image)    
    cX = int(Mnts["m10"] / Mnts["m00"])
    cY = int(Mnts["m01"] / Mnts["m00"])  

    return np.array([cX,cY])

    ## Publish the results
    #try: 
    #  self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
    #except CvBridgeError as e:
    #  print(e)

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


