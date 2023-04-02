#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from exercise2.srv import Walk, WalkResponse


pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1000)

def circle_movement(step):

  twist = Twist()
  twist.linear.x = 0.5

  twist.angular.z = 0.5 #(90 / 360) * 2 * 3.14
	
  return twist

def triangle_movement(step):

  twist = Twist()
  twist.linear.x = 0.5
  step = step % 20

  if step % 5 == 0:
    twist.linear.x = 0
    twist.angular.z = 2.093 #(90 / 360) * 2 * 3.14
	
  return twist

def rectangle_movement(step):

  twist = Twist()
  twist.linear.x = 0.5
  step = step % 20

  if step % 5 == 0:
    twist.linear.x = 0
    twist.angular.z = 1.57 #(90 / 360) * 2 * 3.14
	
  return twist
  
def random_movement(step):

  twist = Twist()
  import random
  
  twist.linear.x = random.uniform(0, 1)
  twist.angular.z = random.uniform(-3.14, 3.14) 
  
  return twist 
  
def handle_walk(req):
  rospy.loginfo("Walking type: %s for %d seconds.", req.type, req.duration)

  r = rospy.Rate(1)

  step = 0.0
  time = req.duration
  
  while time > 0:
    
    if req.type == "circle":
      twist = circle_movement(step)
      pub.publish(twist)
    elif req.type == "triangle":
      twist = triangle_movement(step)
      pub.publish(twist)
    elif req.type == "rectangle":
      twist = rectangle_movement(step)
      pub.publish(twist)
    else:
      twist = random_movement(step)
      pub.publish(twist)
    time = time - 1
    step = step + 1.0
    r.sleep()
    

  return WalkResponse(req.type)	

def walk():

 # pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1000)
  # For the turtle simulation map the topic to /turtle1/cmd_vel
  # For the turtlebot simulation and Turtlebot map the topic to /cmd_vel_mux/input/navi
  #rospy.init_node('movement')
  
  rospy.init_node('walk_service_node')
  s = rospy.Service('walk', Walk, handle_walk)
  rospy.loginfo("Ready to give walk directions.")
  rospy.spin()

if __name__ == '__main__':
    walk()
