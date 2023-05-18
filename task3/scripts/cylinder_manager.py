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
        greet_pose = self.approach_manager.get_object_greet_pose(cylinder_pose)
        self.send_goal_pose(greet_pose)
        

    def extend_camera(self):
        time.sleep(0.5)
        self.arm_manager.arm_movement_pub.publish(self.arm_manager.find_face)
        time.sleep(5)

    def retract_camera(self):
        time.sleep(0.5)
        self.arm_manager.arm_movement_pub.publish(self.arm_manager.retract)
        time.sleep(5)

    def calculate_greet_point(self,cylinder_pose, safe_distance):
        current_point = self.current_robot_pose.position

        # print(current_point)

        target_point = cylinder_pose.position

        # # Calculate the distance between the current point and the se point
        # dx = target_point.x - current_point.x
        # dy = target_point.y - current_point.y
        # dz = target_point.z - current_point.z
        # distance = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

        # # Calculate the unit vector from the current point to the target point
        # if distance == 0:
        #     rospy.logerr("Target point is the same as the current point")
        #     return None
        # unit_vector = Point(dx / distance, dy / distance, dz / distance)

        # # Calculate the point that is 0.3m closer to the current point
        # new_distance = distance - safe_distance
        # new_point = Point(current_point.x + new_distance * unit_vector.x,
        #                   current_point.y + new_distance * unit_vector.y,
        #                   current_point.z + new_distance * unit_vector.z)
        point2 = target_point
        slope = (point2.y - current_point.y) / (point2.x - current_point.x)
        y_intercept = point2.y - slope * point2.x

        if slope == 0:
            # Line is parallel to the x-axis
            nearest_point = Point()
            nearest_point.x = y_intercept
            nearest_point.y = current_point.y
        elif abs(slope) == float('inf'):
            # Line is parallel to the y-axis
            nearest_point = Point()
            nearest_point.x = current_point.x
            nearest_point.y = y_intercept
        else:
            # Line is neither parallel to the x-axis nor the y-axis
            x = (current_point.y - y_intercept) / slope
            y = slope * current_point.x + y_intercept
            nearest_point = Point()
            nearest_point.x = x
            nearest_point.y = y
        
        # Calculate the distance between the current point and the se point
        dx = target_point.x - nearest_point.x
        dy = target_point.y - nearest_point.y
        dz = target_point.z - nearest_point.z
        distance = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

        # Calculate the unit vector from the current point to the target point
        if distance == 0:
            rospy.logerr("Target point is the same as the current point")
            return None
        unit_vector = Point(dx / distance, dy / distance, dz / distance)

        # Calculate the point that is 0.3m closer to the current point
        new_distance = distance - safe_distance
        new_point = Point(nearest_point.x + new_distance * unit_vector.x,
                        nearest_point.y + new_distance * unit_vector.y,
                        nearest_point.z + new_distance * unit_vector.z)

        #self.publish_marker(new_point, color=ColorRGBA(0, 0, 1, 1))
        print("GREEEEEEEEEEEEEEET")
        return new_point
    
    def send_goal_pose(self, pose):

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = pose.position.x
        goal.target_pose.pose.position.y = pose.position.y

        goal.target_pose.pose.orientation.x = pose.orientation.x
        goal.target_pose.pose.orientation.y = pose.orientation.y
        goal.target_pose.pose.orientation.z = pose.orientation.z
        goal.target_pose.pose.orientation.w = pose.orientation.w

        #rospy.loginfo("Sending next goal with xpose: %s" % str(pose))
        print("Cylinder greet position:",pose.position)
        self.client.send_goal(goal)

        wait_result = self.client.wait_for_result()

        if not wait_result:
            rospy.logerr("Not able to set goal.")
        else:
            res = self.client.get_state()
            #rospy.loginfo(str(res))
            return res
    
    def find_prisoner(self):
        
        for cylinder in self.cylinders:
            # Approach cylinder
            self.approach_cylinder(cylinder.pose)

            # Find face on cylinder
            self.extend_camera()

            face_image = self.cylinder_face_manager.find_faces()

            # cv2.imshow("ImWindow", face_image)
            # cv2.waitKey(0)

            # Compare face to poster
            # If face corresponds to poster return true --> we found the robber
            # TODO
            prisoner_found = False 
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


    pose = Pose()
    pose.position.x = 0.5378066572479696
    pose.position.y = 0.33111667322396626
    pose.position.z = 0.2870562880963201
    cyl = Cylinder("blue", pose)
    cm = CylinderManager(None, [cyl])
    cm.find_prisoner()


if __name__ == '__main__':
    main()