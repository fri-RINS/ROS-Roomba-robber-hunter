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

goals = [
    {'x': 0.15, 'y': -1.8},
    {'x': 2.0, 'y': -1.899},
    {'x': 1.55, 'y': 0.05},
    {'x': 0.25, 'y': 1.9},
    {'x': -0.25, 'y': -0.3},
    # add more goals as needed
]

n_goals = len(goals)
cv_map = None
map_resolution = None
map_transform = TransformStamped()

cmd_vel_pub = None


def rotate(angle, speed):   
    
    # Set the rate at which to publish the command (in Hz)
    rate = rospy.Rate(10)

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

def explore_goals(client):

    for i, goal in enumerate(goals):
        goal_x = goal['x']
        goal_y = goal['y']

        # ROTATE THE ROBOT
        rotate(1,0.5)


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
                
                print("Prišel na goal {i}: x: {goal_x}, y: {goal_y}")                
                client.wait_for_result()
            else:
                rospy.loginfo("Couldn't move robot")

if __name__ == '__main__':
    #try:
    rospy.init_node('task1_goals', anonymous=True)
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    explore_goals(client)

        
      
    #except rospy.ROSInterruptException:
    #    rospy.loginfo("Navigation test finished.")