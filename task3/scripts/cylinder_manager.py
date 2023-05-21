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

class CylinderManager:
    def __init__(self,wanted_poster:Poster, cylinders: list):
        self.current_robot_pose = None
        self.odom_sub = rospy.Subscriber(
        '/amcl_pose', PoseWithCovarianceStamped, self.current_robot_pose_callback)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.cylinder_face_manager = CylinderFaceManager()
        self.arm_manager = Arm_Mover()
        self.speaking_manager = SpeakingManager()
        self.approach_manager = ApproachManager()
        
        self.cmd_vel_pub = rospy.Publisher(
        '/cmd_vel_mux/input/teleop', Twist, queue_size=100)
        self.safe_distance = 0.3
        rospy.sleep(2)

        self.poster = wanted_poster
        self.cylinders = cylinders
        self.icm = ImageCompareManager()

        


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
    
    def approach_cylinder(self, pose):

        cylinder_pose = pose
        x_obj = cylinder_pose.position.x
        y_obj = cylinder_pose.position.y
        greet_pose = self.approach_manager.get_object_greet_pose(x_obj,y_obj)
        self.approach_manager.send_goal_pose(greet_pose)
        

    def extend_camera(self):
        time.sleep(0.5)
        self.arm_manager.arm_movement_pub.publish(self.arm_manager.find_face)
        time.sleep(5)

    def retract_camera(self):
        time.sleep(0.5)
        self.arm_manager.arm_movement_pub.publish(self.arm_manager.retract)
        time.sleep(5)
    
    def find_prisoner(self):
        rospy.loginfo("Retracting camera.")

        self.retract_camera()
        for cylinder in self.cylinders:
            # Approach cylinder
            rospy.loginfo("Approaching cylinder.")
            self.approach_cylinder(cylinder.pose)

            # Find face on cylinder
            rospy.loginfo("Extending camera.")
            self.extend_camera()

            face_image = self.cylinder_face_manager.find_faces()

            # cv2.imshow("ImWindow", face_image)
            # cv2.waitKey(0)

            # Compare face to poster
            # If face corresponds to poster return true --> we found the robber
            # TODO
            prisoner_found = self.icm.compare_faces(face_image, self.wanted_poster.image)

            #prisoner_found = False 
            if prisoner_found:
                rospy.loginfo("We found the prisoner.")
                self.speaking_manager.say_arrest_robber()

                return
            else:
                rospy.loginfo("This is not the prisoner. Let's check another cylinder.")
                self.retract_camera()


def main():
    rospy.init_node('cylinder_manager_node', anonymous=True)
    time.sleep(2)
    rate = rospy.Rate(2)
    image_path1 = '/home/kuznerjaka/catkin_ws/src/hw3/task3/img_test/img_test/image_2.jpg'

    poster_image = cv2.imread(image_path1)
    poster = Poster("blue", poster_image, 100, True)
    
    pose = Pose()
    pose.position.x = 0.5378066572479696
    pose.position.y = 0.33111667322396626
    pose.position.z = 0.2870562880963201
    cyl = Cylinder("blue", pose)
    cm = CylinderManager(poster, [cyl])
    rospy.loginfo("Finding the prisoner.")
    cm.find_prisoner()


if __name__ == '__main__':
    main()