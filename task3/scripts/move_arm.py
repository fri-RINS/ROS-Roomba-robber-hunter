#!/usr/bin/python3

import rospy
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String

class Arm_Mover():
    def __init__(self):

        
        
        self.arm_movement_pub = rospy.Publisher('/turtlebot_arm/arm_controller/command', JointTrajectory, queue_size=1)
        self.arm_user_command_sub = rospy.Subscriber("/arm_command", String, self.new_user_command)

        # Just for controlling wheter to set the new arm position
        self.user_command = None
        self.send_command = False

        # Pre-defined positions for the arm
        self.retract = JointTrajectory()
        self.retract.joint_names = ["arm_shoulder_pan_joint", "arm_shoulder_lift_joint", "arm_elbow_flex_joint", "arm_wrist_flex_joint"]
        self.retract.points = [JointTrajectoryPoint(positions=[0,-1.3,2.2,1],
                                                    time_from_start = rospy.Duration(1))]

        self.extend = JointTrajectory()
        self.extend.joint_names = ["arm_shoulder_pan_joint", "arm_shoulder_lift_joint", "arm_elbow_flex_joint", "arm_wrist_flex_joint"]
        self.extend.points = [JointTrajectoryPoint(positions=[0,0.3,1,0],
                                                    time_from_start = rospy.Duration(1))]
        
        self.right = JointTrajectory()
        self.right.joint_names = ["arm_shoulder_pan_joint", "arm_shoulder_lift_joint", "arm_elbow_flex_joint", "arm_wrist_flex_joint"]
        self.right.points = [JointTrajectoryPoint(positions=[-1.57,0.3,1,0],
                                                    time_from_start = rospy.Duration(1))]
        
        self.find_face = JointTrajectory()
        self.find_face.joint_names = ["arm_shoulder_pan_joint", "arm_shoulder_lift_joint", "arm_elbow_flex_joint", "arm_wrist_flex_joint"]
        self.find_face.points = [JointTrajectoryPoint(positions=[0,0.3,1,-0.5],
                                                    time_from_start = rospy.Duration(1))]
        
        self.wave = JointTrajectory()
        self.wave.joint_names = ["arm_shoulder_pan_joint", "arm_shoulder_lift_joint", "arm_elbow_flex_joint", "arm_wrist_flex_joint"]
        self.wave.points = [
            JointTrajectoryPoint(positions=[0, 0.3, 1, 0],
                                 time_from_start=rospy.Duration(1)),
            JointTrajectoryPoint(positions=[0, 0.3, 1, -0.5],
                                 time_from_start=rospy.Duration(2)),
            JointTrajectoryPoint(positions=[0, 0.3, 1, 0],
                                 time_from_start=rospy.Duration(3))
        ]

    def new_user_command(self, data):
        self.user_command = data.data.strip()
        self.send_command = True

    def update_position(self):
        # Only if we had a new command
        if self.send_command:
            if self.user_command == 'retract':
                self.arm_movement_pub.publish(self.retract)
                print('Retracted arm!')
            elif self.user_command == 'extend':
                self.arm_movement_pub.publish(self.extend)
                print('Extended arm!')
            elif self.user_command == 'right':
                self.arm_movement_pub.publish(self.right)
                print('Right-ed arm!')
            else:
                print('Unknown instruction:', self.user_command)
                return(-1)
            self.send_command = False

    def extend_to_face(self):
        time.sleep(0.5)
        self.arm_movement_pub.publish(self.find_face)
        time.sleep(5)

    def retract_camera(self):
        time.sleep(0.5)
        self.arm_movement_pub.publish(self.retract)
        time.sleep(5)

    def extend_to_park(self):
        time.sleep(0.5)
        self.arm_movement_pub.publish(self.extend)
        time.sleep(5)

    def wave_camera(self):
        time.sleep(0.5)
        self.arm_movement_pub.publish(self.wave)
        time.sleep(5)

if __name__ == "__main__":
    rospy.init_node('arm_mover', anonymous=True)
    am = Arm_Mover()
    time.sleep(.5)
    am.arm_movement_pub.publish(am.retract)
    print('Extend arm!')
    
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        am.update_position()
        r.sleep()
