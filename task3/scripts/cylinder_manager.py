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
from cylinder_face_manager import CylinderFaceManager
import cv2
from move_arm import Arm_Mover
from speaking_manager import SpeakingManager

from approach_manager import ApproachManager
from face_compare import ImageCompareManager
import numpy as np

class CylinderManager:
    def __init__(self,wanted_poster:Poster, cylinders: list,am = None):
        self.current_robot_pose = None
        self.odom_sub = rospy.Subscriber(
        '/amcl_pose', PoseWithCovarianceStamped, self.current_robot_pose_callback)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.cylinder_face_manager = CylinderFaceManager()
        self.arm_manager = Arm_Mover()
        self.speaking_manager = SpeakingManager()
        self.approach_manager = am
        
        self.cmd_vel_pub = rospy.Publisher(
        '/cmd_vel_mux/input/teleop', Twist, queue_size=100)
        self.safe_distance = 0.3
        rospy.sleep(2)

        self.wanted_poster = wanted_poster
        self.wanted_poster.image = self.get_img_from_poster(self.wanted_poster)
        self.cylinders = cylinders
        self.icm = ImageCompareManager()

        
    def get_img_from_poster(self, poster):
        poster_image = poster.image
        poster_image = cv2.cvtColor(poster_image, cv2.COLOR_BGR2RGB)
        poster_image = cv2.cvtColor(poster_image, cv2.COLOR_RGB2BGR)
        return poster_image


    def move_forward(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.2
        self.cmd_vel_pub.publish(twist_msg)
        rospy.sleep(1)
        twist_msg.linear.x = 0.0
        self.cmd_vel_pub.publish(twist_msg)  

    def move_back(self):
        twist_msg = Twist()
        twist_msg.linear.x = -0.1
        self.cmd_vel_pub.publish(twist_msg)
        rospy.sleep(1)
        twist_msg.linear.x = 0.0
        self.cmd_vel_pub.publish(twist_msg)  

    def rotate_right(self):
        twist_msg = Twist()
        twist_msg.angular.z = -0.1
        self.cmd_vel_pub.publish(twist_msg)
        rospy.sleep(1)
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)  

    def rotate_left(self):
        twist_msg = Twist()
        twist_msg.angular.z = 0.2
        self.cmd_vel_pub.publish(twist_msg)
        rospy.sleep(1)
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)  


    def current_robot_pose_callback(self,data):

        self.current_robot_pose = data.pose.pose
        # print(current_robot_pose)
    
    def approach_cylinder(self, pose):

        cylinder_pose = pose
        x_obj = cylinder_pose.position.x
        y_obj = cylinder_pose.position.y
        greet_pose = self.approach_manager.get_object_greet_pose(x_obj,y_obj)
        self.approach_manager.send_goal_pose(greet_pose)

    def approach_cylinder_final(self, pose):

        cylinder_pose = pose
        x_obj = cylinder_pose.position.x
        y_obj = cylinder_pose.position.y
        greet_pose = self.approach_manager.get_object_greet_pose(x_obj,y_obj)
        self.approach_manager.send_goal_pose(greet_pose)
    
    def find_prisoner(self):
        rospy.loginfo("Retracting camera.")
        confidences = []
        #self.retract_camera()
        for cylinder in self.cylinders:
            # Approach cylinder
            rospy.loginfo(f"lets check {cylinder.color} cylinder.")
            rospy.loginfo("Approaching cylinder.")
           
            self.approach_cylinder(cylinder.pose)

            # Find face on cylinder
            rospy.loginfo("Extending camera.")
            self.arm_manager.extend_to_face()

            face_image = self.cylinder_face_manager.find_faces()
            if face_image is None:
                confidence = 0 
            else:
                confidence = self.icm.compare_faces(face_image, self.wanted_poster.image)
            confidences.append(confidence)
            self.arm_manager.retract_camera()



        max_conf = np.argmax(confidences)
        max_cylinder = self.cylinders[max_conf]
        self.approach_cylinder_final(max_cylinder.pose)
        rospy.loginfo("We found the prisoner.")
        self.speaking_manager.say_arrest_robber()

        return
            


def main():
    rospy.init_node('cylinder_manager_node', anonymous=True)
    #time.sleep(2)
    rate = rospy.Rate(2)
    image_path1 = '/home/team_predictive_pirates/ROS/src/hw3/task3/img_test/image1.jpg'

    poster_image = cv2.imread(image_path1)
    poster = Poster("blue", poster_image, 100, True)
    
    pose = Pose()
    pose.position.x = 3.265
    pose.position.y = 0.258
    pose.position.z = 0.2870562880963201
    cyl = Cylinder("red", pose)
    pose = Pose()
    pose.position.x = 2.43
    pose.position.y = 2.66
    pose.position.z = 0.2870562880963201
    cyl2 = Cylinder("yellow", pose)
    am = ApproachManager()
    cm = CylinderManager(poster, [cyl,cyl2],am=am)
    rospy.loginfo("Finding the prisoner.")
    cm.find_prisoner()


if __name__ == '__main__':
    main()