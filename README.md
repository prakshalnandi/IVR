# IVR
IVR Assignment (Robot Vision and Control)

This is a description of how to run different parts of the assignments.

Joint Estimation :
Image1 and Image2 python files captures blobs from the images and publishes data for each spheres including target. Image Subscriber receives all these data and converts them in 3d positions, estimates joints angles based on algorithms and publishes them. You can use joint_move script to move robot joints as per To run joint estimation part, run following scripts in this sequence and use /Image/Joints topic to track the estimated values.
* image1.py
* image2.py    
* image_subscriber_node.py    
* joint_move.py

Target detection:

Forward Kinematics :

Robot Control :
A new node is created for robot PID control, which takes input of either estimated joint values or actual robot joint states, current target, end effector positions and moves robot with the help of inverse kinematics.Currently, we have subscibed to actual robot joint states, if you want to use estimated joints, please uncomment '/Image/joints' topic subscription (and its related callback) and comment robot joint states subscription '/robot/joint_states'. Run following nodes in order :
* image1.py    
* image2.py    
* image_subscriber_node.py  
* robot_control.py

Joint Estimation black blobs :
`image1_black.py` and `image2_black.py` python files captures blobs from the images and publishes data for each spheres including target. Image Subscriber is the same as for the earlier joint angle estimation task. It receives all these data and converts them in 3d positions, estimates joints angles based on algorithms and publishes them. You can use joint_move script to move robot joints as per To run joint estimation part, run following scripts in this sequence and use /Image/Joints topic to track the estimated values.
* image1_black.py
* image2_black.py    
* image_subscriber_node.py   
