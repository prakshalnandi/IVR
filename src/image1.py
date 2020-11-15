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
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    #self.t0 = rospy.get_time()
    
    #self.robot_joint2_new_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    #self.robot_joint3_new_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    #self.robot_joint4_new_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10) 
    
    self.robot_joint2_pub = rospy.Publisher("cal_joint2", Float64, queue_size=10)
    self.robot_joint4_pub = rospy.Publisher("cal_joint4", Float64, queue_size=10)
    self.joint2 = Float64()
    self.joint4 = Float64()
  
  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    # Uncomment if you want to save the image
    cv2.imwrite('image_copy.png', self.cv_image1)

     #im1=cv2.imshow('window1', self.cv_image1)
    #cv2.waitKey(1)
    
    
    #move joints over
    
    
    self.CalculateJoints(self.cv_image1)
    self.robot_joint2_pub.publish(self.joint2)
    self.robot_joint4_pub.publish(self.joint4)
    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
    except CvBridgeError as e:
      print(e)
      
    #add code to move joints
    #cur_time = np.array([rospy.get_time()])-self.t0
    #joint2new=Float64()
    #joint2new.data= np.pi/2 * np.sin(cur_time * np.pi/15)
    #print(joint2new)
    #joint3new=Float64()
    #joint3new.data= np.pi/2 * np.sin(cur_time * np.pi/18)
    #joint4new=Float64()
    #joint4new.data= np.pi/2 * np.sin(cur_time * np.pi/20)
    #self.robot_joint2_new_pub.publish(joint2new)
    #self.robot_joint3_new_pub.publish(joint3new)
    #self.robot_joint4_new_pub.publish(joint4new)
      
  def CalculateJoints(self,Image):
  
    ret, threshold = cv2.threshold(Image, 100, 255, cv2.THRESH_BINARY)

    dist = self.pixel2meter(threshold)

    cenBlue = dist * self.detectBlueCenter(threshold)
    cenRed = dist * self.detectRedCenter(Image)
    cenGreen = dist * self.detectGreenCenter(threshold)
    cenYellow = dist * self.detectYellowCenter(Image)

    # joint1 = np.arctan2(cenYellow[0] - cenBlue[0],cenYellow[1] - cenBlue[1])
    # joint2 = np.arctan2(cenBlue[0] - cenGreen[0], cenBlue[1] - cenGreen[1]) - joint1
    # joint3 = np.arctan2(cenGreen[0] - cenRed[0], cenGreen[1] - cenRed[1]) - joint1 - joint2

    #joint1 = np.arctan2(cenYellow[0] - cenBlue[0], cenYellow[1] - cenBlue[1])
    self.joint2 = np.arctan2(cenBlue[0] - cenGreen[0], cenBlue[1] - cenGreen[1])
    self.joint4 = np.arctan2(cenGreen[0] - cenRed[0], cenGreen[1] - cenRed[1]) - self.joint2
    
    print(str(self.joint2),str(self.joint4))
       
    
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


