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

class CylinderManager:
    def __init__(self,wanted_poster:Poster, cylinders: list):

        self.poster = wanted_poster
        self.cylinders = cylinders
        self.cylinder_goals = self.init_cylinder_goals()
        self.current_robot_pose = None


        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        
        self.odom_sub = rospy.Subscriber(
        '/amcl_pose', PoseWithCovarianceStamped, self.current_robot_pose_callback)
        self.safe_distance = 0.3

    def current_robot_pose_callback(self,data):

        self.current_robot_pose = data.pose.pose
        # print(current_robot_pose)
    
    def init_cylinder_goals(self):

        for cylinder in self.cylinders:
            cylinder_pose = cylinder.pose
            greet_point = self.calculate_greet_point(cylinder_pose, 0.5)
            #publish_marker(greet_point)
            target_point = cylinder_pose.position

            rospy.loginfo(f"Point to greet face: {greet_point}")

            goal = MoveBaseGoal()
            goal.cylinder_pose.header.frame_id = 'map'
            goal.cylinder_pose.header.stamp = rospy.Time.now()
            goal.cylinder_pose.pose.position.x = greet_point.x
            goal.cylinder_pose.pose.position.y = greet_point.y
            # goal.cylinder_pose.pose.orientation.w = 1.0

            # Calculate the orientation of the goal
            v = [target_point.x - greet_point.x, target_point.y - greet_point.y]
            theta = atan2(v[1], v[0])
            goal.cylinder_pose.pose.orientation.z = math.sin(theta/2.0)
            goal.cylinder_pose.pose.orientation.w = math.cos(theta/2.0)

            self.client.send_goal(goal)
            self.client.wait_for_result()

            if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal reached successfully.")
            else:
                rospy.logwarn("Failed to reach the goal.")


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

        self.publish_marker(new_point, color=ColorRGBA(0, 0, 1, 1))
        print("GREEEEEEEEEEEEEEET")
        return new_point

