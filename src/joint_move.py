#!/usr/bin/env python3


import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64


# Publish data
def move():
  rospy.init_node('joint_pos_cmd', anonymous=True)
  rate = rospy.Rate(30) # 30hz
  # initialize a publisher to send joints' angular position to the robot
  robot_joint2_new_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
  robot_joint3_new_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
  robot_joint4_new_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10) 

    
  t0 = rospy.get_time()
  while not rospy.is_shutdown():
    cur_time = np.array([rospy.get_time()])-t0
       
    # joint2new=Float64()
    # joint2new.data= np.pi/2 * np.sin(cur_time * np.pi/15)
    # #print(joint2new)
    # joint3new=Float64()
    # joint3new.data= np.pi/2 * np.sin(cur_time * np.pi/18)
    # joint4new=Float64()
    # joint4new.data= np.pi/2 * np.sin(cur_time * np.pi/20)

    joint2new=Float64()
    joint2new.data= np.pi/3 * np.sin(cur_time * np.pi/15)
    #print(joint2new)
    joint3new=Float64()
    joint3new.data= np.pi/3 * np.sin(cur_time * np.pi/18)
    joint4new=Float64()
    joint4new.data= np.pi/3 * np.sin(cur_time * np.pi/20)


    robot_joint2_new_pub.publish(joint2new)
    robot_joint3_new_pub.publish(joint3new)
    robot_joint4_new_pub.publish(joint4new)
    
    rate.sleep()



# run the code if the node is called
if __name__ == '__main__':
  try:
    move()
  except rospy.ROSInterruptException:
    pass


