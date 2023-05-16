#!/usr/bin/python3

import rospy
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String

class ImageCompareManager():
    def __init__(self):
        return
        
        
       
if __name__ == "__main__":
    rospy.init_node('compare_image_node', anonymous=True)
    
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()
