#!/usr/bin/python3

from poster_manager import Poster
from marker_manager import Cylinder
import rospy
from goals_task3_improved import MyGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped
from math import atan2
import math
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PoseStamped, Point, TransformStamped
import time
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
import cv2
from move_arm import Arm_Mover
from speaking_manager import SpeakingManager

# Approach prison ring
# Extend arm
# Rotate robot until parking found with ParkingDetector
# Park the robot
# Wave manipulator
# Say goodbye

class ParkingManager:
    def __init__(self,ring_pose: Pose):
        self.current_robot_pose = None
        self.odom_sub = rospy.Subscriber(
        '/amcl_pose', PoseWithCovarianceStamped, self.current_robot_pose_callback)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.arm_manager = Arm_Mover()
        self.speaking_manager = SpeakingManager()
        
        self.cmd_vel_pub = rospy.Publisher(
        '/cmd_vel_mux/input/teleop', Twist, queue_size=100)
    
    def move_forward(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.2
        self.cmd_vel_pub.publish(twist_msg)
        rospy.sleep(1)
        twist_msg.linear.x = 0.0
        self.cmd_vel_pub.publish(twist_msg)  

    def current_robot_pose_callback(self,data):

        self.current_robot_pose = data.pose.pose
        # print(current_robot_pose)
    
    

def main():
    rospy.init_node('parking_manager_node', anonymous=True)
    time.sleep(2)
    rate = rospy.Rate(2)




if __name__ == '__main__':
    main()