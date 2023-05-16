#!/usr/bin/python3

#!/usr/bin/python3

import rospy
from tf2_geometry_msgs import do_transform_pose
import tf2_ros
import cv2
from nav_msgs.msg import OccupancyGrid
import actionlib
from geometry_msgs.msg import PoseStamped, Point, TransformStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray
from exercise2.srv import PlaySound, PlaySoundRequest
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
from geometry_msgs.msg import PointStamped, Vector3, Pose
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from std_msgs.msg import String
import numpy as np
from math import atan2, cos, sin, pi
from functools import partial
from sound_play.libsoundplay import SoundClient
from task3.msg import PosterMessage
from poster_manager import PosterDetector
from poster_manager import Poster
from conversation_manager import ConversationManager
from marker_manager import MarkerManager
from speaking_manager import SpeakingManager




goals = [
    {'x': 0.1, 'y': -1.65},
    {'x': 0.12, 'y': -1.6},
    {'x': 0.1, 'y': -1.5},
    {'x': 1.0, 'y': -1.7},
    {'x': 3.1, 'y': -1.05},
    {'x': 2.35, 'y': 1.85},
    {'x': -1.0, 'y': 1.1}
    # add more goals as needed
]



rings_found = []
cylinders_found=[]

class MyGoal:
    def __init__(self, goal: MoveBaseGoal, type: str, color: str, pose=None):
        self.goal = goal
        self.type = type
        # self.is_completed = False
        self.color = color
        self.pose = pose

    # def makeCompleted(self):
    #     self.is_completed = True

    def get_goal_coordinates(self):

        return self.goal.target_pose.pose.position.x, self.goal.target_pose.pose.position.y


class GoalQueue:
    def __init__(self, goal_points, num_faces):

        self.map_goals = []
        self.face_goals = []
        self.completed_face_goals = []
        self.num_faces = num_faces
        self.greeted_faces = 0
        self.running = True  # When All works is done set to False -> stop the robot
        self.init_map_goals(goal_points)
        #number of items to detect before approaching green
        self.cylinders_to_detect = 3
        self.rings_to_detect = 3
        self.posters = []
        self.mm = MarkerManager()
        self.sm = SpeakingManager()
        self.id_map_goal = 0
        self.current_robot_pose = None
        self.cylinder_colors_to_check = None
        #two cylinders that need to be approached
        self.cylinders_to_approach = None
        #all rings
        self.rings = None

        self.cmd_vel_pub = rospy.Publisher(
        '/cmd_vel_mux/input/teleop', Twist, queue_size=100)
        self.odom_sub = rospy.Subscriber(
        '/amcl_pose', PoseWithCovarianceStamped, self.current_robot_pose_callback)
        self.markers_pub = rospy.Publisher('greet_point_markers', MarkerArray, queue_size=1000)
        self.arm_pub = rospy.Publisher('/arm_command', String, queue_size=10)
        self.start_park_detection_pub = rospy.Publisher('/start_park_detection', String, queue_size=1)

    def init_map_goals(self, init_goal_points):

        for goal_point in init_goal_points:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = goal_point['x']
            goal.target_pose.pose.position.y = goal_point['y']
            goal.target_pose.pose.orientation.w = 1.0

            myGoal = MyGoal(goal, "map", None)
            self.map_goals.append(myGoal)

    def print_goals(self):
        print("Map goals:")
        for goal in self.map_goals:
            print(goal.get_goal_coordinates())
        print("Face goals:")
        for goal in self.face_goals:
            print(goal.get_goal_coordinates())

    def get_next_goal(self):
        if len(self.face_goals) > 0:
            return self.face_goals[0]
        else:
            return self.map_goals[0]

    def check_if_everything_detected(self):
        n_rings = len(self.mm.rings)
        n_cylinder = len(self.mm.cylinders)
        n_faces = len(self.completed_face_goals)

        if n_rings >= self.rings_to_detect and n_cylinder >= self.cylinders_to_detect and n_faces >= self.num_faces:
            print("Detected all rings, cylinders and and approached all faces, Running = False.")
            self.rings = self.mm.rings
            self.cylinders_to_approach = self.find_cylinders(self.cylinder_colors_to_check)
            self.running = False
            
    def find_cylinders(self,colors):

        cylinders_to_visit = []
        for c in colors:
            for cylinder in self.mm.cylinders:
                if cylinder.color == c:
                    cylinders_to_visit.append(cylinder)

        return cylinders_to_visit



        

    def add_face_goal(self, target_pose):
        # Calculate greet point
        greet_point = self.calculate_greet_point(target_pose, 0.5)
        #publish_marker(greet_point)
        target_point = target_pose.position

        rospy.loginfo(f"Point to greet face: {greet_point}")

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = greet_point.x
        goal.target_pose.pose.position.y = greet_point.y
        # goal.target_pose.pose.orientation.w = 1.0

        # Calculate the orientation of the goal
        v = [target_point.x - greet_point.x, target_point.y - greet_point.y]
        theta = atan2(v[1], v[0])
        goal.target_pose.pose.orientation.z = math.sin(theta/2.0)
        goal.target_pose.pose.orientation.w = math.cos(theta/2.0)

        my_face_goal = MyGoal(goal, "face", None, target_pose)
        self.face_goals.append(my_face_goal)
        return

    def complete_map_goal(self, completed_goal):
        self.map_goals.remove(completed_goal)
        new_goal = completed_goal
        self.map_goals.append(new_goal)

    def complete_face_goal(self, completed_goal):
        self.face_goals.remove(completed_goal)
        self.completed_face_goals.append(completed_goal)
        if len(self.completed_face_goals) == self.num_faces:
            rospy.loginfo("All faces greeted.")
    
    def approach_face(self,goal):

        # Send the goal to the move_base action server
        client.send_goal(goal)
        # rospy.loginfo(f"Approaching the face #{num_faces}")
        client.wait_for_result()
        rospy.loginfo("Face was approached.")

        # Remove aprroached face from face_to_approach
        self.mm.face_to_approach = None

    def do_face_goal(self,my_goal):

        self.approach_face(my_goal.goal)

        twist_msg = Twist()
        twist_msg.linear.x = 0.2
        self.cmd_vel_pub.publish(twist_msg)
        rospy.sleep(1)
        twist_msg.linear.x = 0.0
        self.cmd_vel_pub.publish(twist_msg)

        rospy.loginfo("Checking if this is a poster.")
        pd = PosterDetector(my_goal.pose)

        poster = pd.find_poster()

        if poster == None:
            rospy.loginfo("This is a FACE. Let's ask questions.")    
            # SAY HELLO
            self.sm.greet()
            # TODO
            # CONVERSATION
            soundhandle = SoundClient()
        # for testing
            st = ConversationManager()
            response = st.talkToPerson(soundhandle)
            print(f"Response: {response}")

            if isinstance(response,list):
                print("Response is array of cylinder colors!")
                self.cylinder_colors_to_check = response
            
        else:
            rospy.loginfo("This is a POSTER. Will not greet.")
            self.posters.append(poster)

        self.complete_face_goal(my_goal)
        return
    
    def do_map_goal(self,my_goal):
        client.send_goal(my_goal.goal)

        goal_x, goal_y = my_goal.get_goal_coordinates()

        rospy.loginfo(f"Sending to coordinates: x: {goal_x}, y: {goal_y}")
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            if client.get_result():
                rospy.loginfo(f"Reached goal {self.id_map_goal}: x: {goal_x}, y: {goal_y}")
                goal_queue.complete_map_goal(my_goal)
                self.id_map_goal += 1
                self.rotate(1, 0.3)
            else:
                rospy.loginfo("Couldn't move robot")

        return
    def rotate(self,angle, speed):

        # Set the rate at which to publish the command (in Hz)
        rate = rospy.Rate(2)

        # Create a Twist message to hold the command
        cmd_vel = Twist()

        # Set the angular velocity to rotate around the z-axis
        cmd_vel.angular.z = speed * angle  # in radians/sec

        # Set the duration to rotate for one circle
        t0 = rospy.Time.now()
        duration = rospy.Duration.from_sec(
            6.28/speed)  # 6.28 radians = 360 degrees

        rospy.loginfo("Start rotating.")

        # Loop until the duration has passed
        while rospy.Time.now() - t0 < duration:
            self.cmd_vel_pub.publish(cmd_vel)
            rate.sleep()
            if self.mm.face_to_approach:
                break

        # Stop the robot after the duration has passed
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
        rospy.loginfo("Finished rotating.")

    def calculate_greet_point(self,target_pose, safe_distance):
        current_point = self.current_robot_pose.position

        # print(current_point)

        target_point = target_pose.position

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
    def publish_marker(self,position, color=ColorRGBA(1, 0, 0, 1)):

        marker_array = MarkerArray()
        marker = Marker()
        marker.header.stamp = rospy.Time(0)
        marker.header.frame_id = 'map'
        marker.pose.position = position
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.lifetime = rospy.Duration.from_sec(10)
        marker.id = 1
        marker.scale = Vector3(0.1, 0.1, 0.1)
        marker.color = color
        marker_array.markers.append(marker)

        self.markers_pub.publish(marker_array)

    def current_robot_pose_callback(self,data):

        current_robot_pose = data.pose.pose
        # print(current_robot_pose)



def explore_goals1(goal_queue:GoalQueue):
    while goal_queue.running:

        if goal_queue.mm.face_to_approach:
            goal_queue.add_face_goal(goal_queue.mm.face_to_approach)
            goal_queue.mm.face_to_approach = None

        next_goal = goal_queue.get_next_goal()

        if next_goal.type == "map":
            goal_queue.do_map_goal(next_goal)
        elif next_goal.type == "face":
            goal_queue.do_face_goal(next_goal)
    






if __name__ == '__main__':
    # try:
    rospy.init_node('task1_goals', anonymous=True)
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Init map goals
    goal_points = [
        {'x': 0.1, 'y': -1.65},
        {'x': 0.0, 'y': 0.0},
        {'x': 1.0, 'y': -1.7}
    ]

    goal_points2 = [
        #{'x': 0, 'y': -1},
        #{'x': 1, 'y': 0},
        #{'x': 2.5, 'y': 1.3},
        {'x': 1, 'y': 2.5},
        #{'x': 0.12, 'y': -1.6},
        #{'x': 0.1, 'y': -1.5},
        #{'x': 1.0, 'y': -1.7},
        #{'x': 3.1, 'y': -1.05},
        #{'x': 2.35, 'y': 1.85},
        #{'x': -1.0, 'y': 1.1}
        
        # add more goals as needed
    ]
    # add more goals as needed
    goal_queue = GoalQueue(goal_points2, num_faces=3)

    goal_queue.print_goals()

    rate = rospy.Rate(1)
    explore_goals1(goal_queue)