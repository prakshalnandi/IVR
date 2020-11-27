#!/usr/bin/env python3

import sys
import numpy as np
import roslib
import rospy
import cv2
import time
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError

class robot_controller:

    def __init__(self):
  
        # initialize the node named image_processing
        rospy.init_node('robot_control', anonymous=True)
        ####extra
        
        ####extra
        # initialize errors
        self.time_previous_step = np.array([rospy.get_time()], dtype='float64')
        print("initial time",self.time_previous_step)
        self.time_previous_step2 = np.array([rospy.get_time()], dtype='float64')
        self.time_previous_step = rospy.get_time()
        print("initial time",self.time_previous_step)
        self.q_previous = np.array([0,0,0,0])
        self.time_trajectory = rospy.get_time()
        self.oldpos = self.trajectory()
        # initialize error and derivative of error for trajectory tracking
        self.error = np.array([0.0, 0.0, 0.0], dtype='float64')
        self.error_d = np.array([0.0, 0.0, 0.0], dtype='float64')
        #self.imagetargetsub = rospy.Subscriber("/Image/target", Float64MultiArray, self.callback)
        self.imagetargetsub = rospy.Subscriber("/target/joint_states", JointState, self.callback)
        self.endeffectorsub = rospy.Subscriber("/Image/endeffector", Float64MultiArray, self.callback3)
        self.jointactualsub = rospy.Subscriber("/robot/joint_states", JointState, self.callbackJointStates)
        #self.jointvisionsub = rospy.Subscriber("/Image/Joints", Float64MultiArray, self.callbackVisionJoints)
        #self.joint1stsub = rospy.Subscriber("/Image/Joints", Float64MultiArray, self.callbackVisionJoints)
        self.endeffector = Float64MultiArray()
        self.target = Float64MultiArray()
        self.previousjoints = Float64MultiArray()
        # initialize a publisher to send joints' angular position to the robot
        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
        
    # Define a circular trajectory
    def trajectory(self):

        cur_time = np.array([rospy.get_time() - self.time_trajectory])
        x_d = float(2.5 * np.cos(cur_time * np.pi / 15))
        y_d = float(2.5 * np.sin(cur_time * np.pi / 15))
        z_d = float(1 * np.sin(cur_time * np.pi / 15))

        return np.array([x_d, y_d,z_d],dtype='float64')

    # Calculate the robot Jacobian
    def calculate_jacobian(self, angles):
        #joints = self.detect_joint_angles(image)
        theta1 = angles[0]
        theta2 = angles[1]
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

        #FK
        X = 3.5 * s1 * s2 * c3 - 3 * c2 * s1 * s4 + 3 * c4 * (s1 * s2 * c3 - c1 * s3) + 3.5 * c1 * s3
        Y = 3.5 * s1 * s3 - 3.5 * s2 * c3 * c1 + 3 * c4 * (s1 * s3 - s2 * c3 * c1) - 3 * c1 * c2 * s4
        Z = -3 * s2 * s4 + 3 * c3 * c4 * c2 + 3.5 * c3 * c2 + 2.5
                
        jacobian = np.array(
            [[3 * c1 * c3 * c4 * s2 - 3 * s1 * s3 * c4 + 3 * c1 * c2 * c4 + 3.5 * s2 * c1 * c3 - 3.5 * s1 * s3,
              3 * s1 * c2 * c3 * c4 - 3 * s1 * s2 * s4 + 3.5 * s1 * c2 * c3,
              -3 * s1 * s2 * s3 * c4 + 3 * c1 * c3 * c4 - 3.5 * s1 * s2 * s3 + 3.5 * c1 * c3,
              -3 * s1 * s2 * s4 * c3 - 3 * c1 * s4 * s3 + 3 * c2 * c4 * s1],
             [3 * c3 * c4 * s1 * s2 + 3 * c2 * s1 * s4 + 3.5 * c3 * s1 * s2 + 3 * c1 * c4 * s3 + 3.5 * c1 * s3,
              -3 * c1 * c2 * c3 * c4 + 3 * c1 * s2 * s4 - 3.5 * c1 * c2 * c3,
              3 * c1 * c4 * s2 * s3 + 3.5 * c1 * s2 * s3 + 3 * c3 * c4 * s1 + 3.5 * c3 * s1,
              3 * c1 * c3 * s2 * s4 - 3 * c1 * c2 * s4 - 3 * s1 * s3 * s4],
             [0,
              -3 * c3 * c4 * s2 - 3 * c2 * s4 - 3.5 * c3 * s2,
              -3 * c2 * c4 * s3 - 3.5 * c2 * s3,
              -3 * c2 * c3 * s4 - 3 * c4 * s2]])

        return jacobian

    # unc def control_closed(self, image):
    def control_closed(self):
        # P gain
        K_p = np.array([[5, 0, 0], [0, 5, 0], [0, 0, 5]])
        # D gain
        K_d = np.array([[0.5, 0, 0], [0, 0.5, 0], [0, 0, 0.5]])
        # estimate time step
        # unc cur_time = np.array([rospy.get_time()])
        cur_time = rospy.get_time()
        dt	 = cur_time - self.time_previous_step
        if(dt > 1) :
          dt = 0.04
        if(dt == 0.0) :
          dt = 0.04
        self.time_previous_step = cur_time

        #print("Target",self.target)
        #print("End Effector",self.endeffector)
        pos_d = np.array(self.target)
        pos = np.array(self.endeffector)
        
        # estimate derivative of error
        self.error_d = ((pos_d - pos) - self.error) / dt
        #self.error_d = (pos_d - self.error) / dt
        # estimate error
        self.error = pos_d - pos
        #self.error = pos_d
        # estimate initial value of joints'
        J_inv = np.linalg.pinv(self.calculate_jacobian(self.q_previous))  # calculating the psudeo inverse of Jacobian

        dq_d = np.dot(J_inv, (np.dot(K_d, self.error_d.transpose()) + np.dot(K_p,self.error.transpose())))  # control input (angular velocity of joints)
        #dq_d = np.dot(J_inv, self.error_d.transpose())
        #print("dq_d",dq_d)
        #print("dt",dt)
        q_d = self.q_previous + (dt * dq_d)  # control input (angular position of joints)
        self.q_previous = q_d
        #print("previous",self.q_previous)
        return q_d
     
    #End effector Values   
    def callback3(self, data):
        self.endeffector = data.data
    
    #Values of Actual Joints  
    def callbackJointStates(self, data):
        self.q_previous = data.position
    
    #Values of Joints from vision    
    def callbackVisionJoints(self, data):
        self.q_previous[1] = data.data[0]        
        self.q_previous[2] = data.data[1]
        self.q_previous[3] = data.data[2]
        
    #Actual value of 1st Joint    
    def callback1stJoint(self, data):        
        self.q_previous[0] = data

    def callback(self, data):
        
        # send control commands to joints
        
        self.target = data.position
        q_d = self.control_closed()
        self.joint1 = Float64()
        self.joint1.data = q_d[0]
        self.joint2 = Float64()
        self.joint2.data = q_d[1]
        self.joint3 = Float64()
        self.joint3.data = q_d[2]
        self.joint4 = Float64()
        self.joint4.data = q_d[3]

        #print("joints",q_d)
        
        # Publish the results
        try:
            self.robot_joint1_pub.publish(self.joint1)
            self.robot_joint2_pub.publish(self.joint2)
            self.robot_joint3_pub.publish(self.joint3)
            self.robot_joint4_pub.publish(self.joint4)
        except CvBridgeError as e:
            print(e)

# call the class
def main(args):
    #print("Hello")
    ic = robot_controller()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
    
    # run the code if the node is called


if __name__ == '__main__':
    main(sys.argv)


