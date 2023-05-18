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
from parking_detector import ParkingDetector

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
    
    def extend_camera(self):
        time.sleep(0.5)
        self.arm_manager.arm_movement_pub.publish(self.arm_manager.extend)
        #time.sleep(5)

    def wave_arm(self):
        time.sleep(0.5)
        self.arm_manager.arm_movement_pub.publish(self.arm_manager.wave)
        time.sleep(2)
        

    def send_park_goal(self, park_goal):
        print("Parking the robot...")

    

    def approach_ring(self):
        print("Approaching prison")


    def current_robot_pose_callback(self,data):

        self.current_robot_pose = data.pose.pose
        # print(current_robot_pose)
    
    def park(self):
        # Approach prison ring
        self.approach_ring()
        # Extend arm
        self.extend_camera()
        # Rotate robot until parking found with ParkingDetector
        park_detector = ParkingDetector()
        park_goal = park_detector.find_rough_parking()
        # Rough parking
        self.send_park_goal(park_goal)
        # Fine parking
        park_detector.fine_parking()
        # Wave manipulator
        self.wave_arm()
        # Say goodbye
        self.speaking_manager.say_goodbye()
        

def main():
    rospy.init_node('parking_manager_node', anonymous=True)
    time.sleep(2)
    rate = rospy.Rate(2)
    pm = ParkingManager(None)
    




if __name__ == '__main__':
    main()