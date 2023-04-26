#!/usr/bin/env python

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
import numpy as np
from math import atan2
from functools import partial




goals = [
    {'x': 0.1, 'y': -1.65},
    #{'x': 0.12, 'y': -1.6},
    #{'x': 0.1, 'y': -1.5},
    {'x': 1.0, 'y': -1.7},
    {'x': 3.1, 'y': -1.05},
    {'x': 2.35, 'y': 1.85},
    # add more goals as needed
]

goalArray = []



n_goals = len(goals)
cv_map = None
map_resolution = None
map_transform = TransformStamped()
markers_pub = None
cmd_vel_pub = None
face_marker_sub = None
ring_marker_sub = None
odom_sub = None
num_faces = 1
face_to_approach = None
ring_to_approach = None
current_robot_pose = None

class MyGoal:
    def __init__(self, goal: MoveBaseGoal, type: str):
        self.goal = goal
        self.type = type
        #self.is_completed = False

    # def makeCompleted(self):
    #     self.is_completed = True

    def get_goal_coordinates(self):

        return self.goal.target_pose.pose.position.x, self.goal.target_pose.pose.position.y


class GoalQueue:
    def __init__(self, goal_points, num_faces):
        self.map_goals = []
        self.face_goals = []
        self.ring_goals = []
        self.completed_face_goals = []
        self.completed_ring_goals = []
        self.num_faces = num_faces
        self.greeted_faces = 0
        self.can_approach_ring = False # True only for testing
        self.running = True # When All works is done set to False -> stop the robot
        self.init_map_goals(goal_points)

    def init_map_goals(self, init_goal_points):    

        for goal_point in init_goal_points:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = goal_point['x']
            goal.target_pose.pose.position.y = goal_point['y']
            goal.target_pose.pose.orientation.w = 1.0

            myGoal = MyGoal(goal, "map")
            self.map_goals.append(myGoal)

    def print_goals(self):
        print("Map goals:")
        for goal in self.map_goals:
            print(goal.get_goal_coordinates())
        print("Face goals:")
        for goal in self.face_goals:
            print(goal.get_goal_coordinates())

    def get_next_goal(self):
        if len(self.ring_goals) > 0 and self.can_approach_ring == True:
            return self.ring_goals[0]
        elif len(self.face_goals) > 0:
            return self.face_goals[0]
        else:
            return self.map_goals[0]

    
    def add_face_goal(self, target_pose):
        # Calculate greet point
        greet_point = calculate_greet_point(target_pose, 0.3)
        publish_marker(greet_point)
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

        my_face_goal = MyGoal(goal, "face")
        self.face_goals.append(my_face_goal)
        return 

    def add_ring_goal(self, target_pose):
        # Calculate greet point
        greet_point = calculate_greet_point(target_pose, 0.2)
        publish_marker(greet_point)
        target_point = target_pose.position

        rospy.loginfo(f"Point to apprach ring: {greet_point}")

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = greet_point.x
        goal.target_pose.pose.position.y = greet_point.y
        # goal.target_pose.pose.orientation.w = 1.0

        v = [target_point.x - greet_point.x, target_point.y - greet_point.y]
        theta = atan2(v[1], v[0])
        goal.target_pose.pose.orientation.z = math.sin(theta/2.0)
        goal.target_pose.pose.orientation.w = math.cos(theta/2.0)

        my_ring_goal = MyGoal(goal, "ring")
        self.ring_goals.append(my_ring_goal)
    
    def complete_map_goal(self, completed_goal):
        self.map_goals.remove(completed_goal)
        new_goal = completed_goal
        self.map_goals.append(new_goal)

    def complete_face_goal(self, completed_goal):
        self.face_goals.remove(completed_goal)
        self.completed_face_goals.append(completed_goal) 
        if len(self.completed_face_goals) == self.num_faces:
            rospy.loginfo("All faces greeted.")
            self.can_approach_ring = True

    def complete_ring_goal(self, completed_goal):
        self.running = False
        return   


def current_robot_pose_callback(data):
    global current_robot_pose

    current_robot_pose = data.pose.pose
    #print(current_robot_pose)

def publish_marker(position):

    marker_array = MarkerArray()
    marker = Marker()
    marker.header.stamp = rospy.Time(0)
    marker.header.frame_id = 'map'
    marker.pose.position = position
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.frame_locked = False
    marker.lifetime = rospy.Duration.from_sec(10)
    marker.id = 1
    marker.scale = Vector3(0.1, 0.1, 0.1)
    marker.color = ColorRGBA(1, 0, 0, 1)
    marker_array.markers.append(marker)

    markers_pub.publish(marker_array)

def calculate_greet_point(target_pose, safe_distance):
    global current_point
    current_point = current_robot_pose.position

    print(current_point)

    target_point = target_pose.position

    # Calculate the distance between the current point and the se point
    dx = target_point.x - current_point.x
    dy = target_point.y - current_point.y
    dz = target_point.z - current_point.z
    distance = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

    # Calculate the unit vector from the current point to the target point
    if distance == 0:
        rospy.logerr("Target point is the same as the current point")
        return None
    unit_vector = Point(dx / distance, dy / distance, dz / distance)

    # Calculate the point that is 0.3m closer to the current point
    new_distance = distance - safe_distance
    new_point = Point(current_point.x + new_distance * unit_vector.x,
                      current_point.y + new_distance * unit_vector.y,
                      current_point.z + new_distance * unit_vector.z)
    return new_point

def rotate(angle, speed):   
    
    # Set the rate at which to publish the command (in Hz)
    rate = rospy.Rate(2)

    # Create a Twist message to hold the command
    cmd_vel = Twist()

    # Set the angular velocity to rotate around the z-axis
    cmd_vel.angular.z = speed * angle  # in radians/sec

    # Set the duration to rotate for one circle
    t0 = rospy.Time.now()
    duration = rospy.Duration.from_sec(6.28/speed)  # 6.28 radians = 360 degrees
    
    rospy.loginfo("Start rotating.")

    # Loop until the duration has passed
    while rospy.Time.now() - t0 < duration:
        cmd_vel_pub.publish(cmd_vel)
        rate.sleep()
        if face_to_approach or ring_to_approach:
            break

    # Stop the robot after the duration has passed
    cmd_vel.angular.z = 0.0
    cmd_vel_pub.publish(cmd_vel)
    rospy.loginfo("Finished rotating.")



def map_callback(msg_map):
    #global cv_map, map_resolution, map_transform
    
    size_x = msg_map.info.width
    size_y = msg_map.info.height

    if size_x < 3 or size_y < 3:
        rospy.loginfo("Map size is only x: %d, y: %d. Not running map to image conversion", size_x, size_y)
        return

    # resize cv image if it doesn't have the same dimensions as the map
    if cv_map is None or cv_map.shape[:2] != (size_y, size_x):
        cv_map = np.zeros((size_y, size_x), dtype=np.uint8)

    map_resolution = msg_map.info.resolution
    map_transform.transform.translation.x = msg_map.info.origin.position.x
    map_transform.transform.translation.y = msg_map.info.origin.position.y
    map_transform.transform.translation.z = msg_map.info.origin.position.z

    map_transform.transform.rotation = msg_map.info.origin.orientation

    map_msg_data = np.array(msg_map.data, dtype=np.int8)

    cv_map_data = cv_map.data

    # We have to flip around the y axis, y for image starts at the top and y for map at the bottom
    size_y_rev = size_y - 1

    for y in range(size_y_rev, -1, -1):
        idx_map_y = size_x * (size_y - y)
        idx_img_y = size_x * y

        for x in range(size_x):
            idx = idx_img_y + x

            if map_msg_data[idx_map_y + x] == -1:
                cv_map_data[idx] = 127
            elif map_msg_data[idx_map_y + x] == 0:
                cv_map_data[idx] = 255
            elif map_msg_data[idx_map_y + x] == 100:
                cv_map_data[idx] = 0

def greet():
    srv = PlaySoundRequest()
    srv.message = "Hello there"
    # call the service
    rospy.loginfo("Saying hello")
    response = sound_client(srv)
    rospy.sleep(1)  


def approach_and_greet(goal):
    global face_to_approach
    global num_faces

    

    # Send the goal to the move_base action server
    client.send_goal(goal)
    # rospy.loginfo(f"Approaching the face #{num_faces}")
    client.wait_for_result()
    rospy.loginfo("Face was approached.")

    # SAY HELLO
    greet()

    # Remove aprroached face from face_to_approach
    face_to_approach = None

    num_faces += 1

def execute_ring(my_goal):
    goal = my_goal.goal

    # Send the goal to the move_base action server
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("Ring was approached.")

    # TODO PARKING

def face_marker_callback(data):
    global face_to_approach
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)

    latest_marker = data.markers[-1]
    latest_marker_pose = latest_marker.pose

    face_x = latest_marker_pose.position.x
    face_y = latest_marker_pose.position.y
    face_z = latest_marker_pose.position.z

    rospy.loginfo(f"Face to apprach: x: {face_x}, y: {face_y}, z: {face_z}")
    face_to_approach = latest_marker_pose

def ring_marker_callback(data):
    global ring_to_approach

    latest_ring = data.markers[-1]
    latest_ring_pose = latest_ring.pose
    latest_ring_color = latest_ring.color

    if latest_ring_color == ColorRGBA(0, 1, 0, 1):
        ring_to_approach = latest_ring_pose
    else:
        rospy.loginfo("Ring is not green, so will not approach")
    
def explore_goals(client):

    i = 0
    while True:

        if face_to_approach is not None:
            approach_and_greet(face_to_approach)
        else:
            rotate(1,0.3) 

        goal = goals[i % len(goals)]

        goal_x = goal['x']
        goal_y = goal['y']        


        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_x
        goal.target_pose.pose.position.y = goal_y
        goal.target_pose.pose.orientation.w = 1.0

        client.send_goal(goal)
        rospy.loginfo(f"Sending to coordinates: x: {goal_x}, y: {goal_y}")
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            if client.get_result():
                rospy.loginfo(f"Reached goal {i}: x: {goal_x}, y: {goal_y}")           
            else:
                rospy.loginfo("Couldn't move robot")
    
        i += 1

def do_map_goal(my_goal, goal_queue):

    client.send_goal(my_goal.goal)

    goal_x, goal_y = my_goal.get_goal_coordinates()

    rospy.loginfo(f"Sending to coordinates: x: {goal_x}, y: {goal_y}")
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        if client.get_result():
            rospy.loginfo(f"Reached goal (TODO ID): x: {goal_x}, y: {goal_y}")  
            goal_queue.complete_map_goal(my_goal)
            rotate(1,0.3)
        else:
            rospy.loginfo("Couldn't move robot")

    return

def do_face_goal(my_goal, goal_queue):
    approach_and_greet(my_goal.goal)
    qoal_queue.complete_face_goal(my_goal)
    return

def do_ring_goal(my_goal, goal_queue):
    print("DO RING GOAL")
    execute_ring(my_goal)
    goal_queue.complete_ring_goal(my_goal)
    return

def explore_goals1(client, goal_queue):
    global face_to_approach
    global ring_to_approach
    while goal_queue.running:

        if face_to_approach:
            goal_queue.add_face_goal(face_to_approach)
            face_to_approach = None
        if ring_to_approach:
            goal_queue.add_ring_goal(ring_to_approach)
            ring_to_approach = None

        next_goal = goal_queue.get_next_goal()

        if next_goal.type == "map":
            do_map_goal(next_goal, goal_queue)
        elif next_goal.type == "face":
            do_face_goal(next_goal, goal_queue)
        elif next_goal.type == "ring":
            do_ring_goal(next_goal, goal_queue)
      

if __name__ == '__main__':
    #try:
    rospy.init_node('task1_goals', anonymous=True)
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    # Init map goals
    goal_points = [
    {'x': 0.1, 'y': -1.65},
    {'x': 0.0, 'y': 0.0},
    {'x': 1.0, 'y': -1.7}
    ]

    goal_points2 = [
    {'x': 0.1, 'y': -1.65},
    #{'x': 0.12, 'y': -1.6},
    #{'x': 0.1, 'y': -1.5},
    {'x': 1.0, 'y': -1.7},
    {'x': 3.1, 'y': -1.05},
    {'x': 2.35, 'y': 1.85},
    {'x': 0, 'y': 0},
    # add more goals as needed
    ]
    # add more goals as needed
    qoal_queue = GoalQueue(goal_points2, num_faces=3)

    cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=100)
    odom_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, current_robot_pose_callback)
    markers_pub = rospy.Publisher('face_markers', MarkerArray, queue_size=1000)

    face_marker_sub = rospy.Subscriber("face_markers", MarkerArray, face_marker_callback, queue_size=10)
    ring_marker_sub = rospy.Subscriber("ring_markers", MarkerArray, ring_marker_callback, queue_size=10)
    sound_client = rospy.ServiceProxy('play_sound', PlaySound)


    
    qoal_queue.print_goals()

    rate = rospy.Rate(1)
    #explore_goals(client)
    explore_goals1(client, qoal_queue)

        
      
    #except rospy.ROSInterruptException:
    #    rospy.loginfo("Navigation test finished.")