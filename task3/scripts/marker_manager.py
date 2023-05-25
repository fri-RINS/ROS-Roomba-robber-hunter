#!/usr/bin/python3

import rospy
from std_msgs.msg import ColorRGBA
from speaking_manager import SpeakingManager
from visualization_msgs.msg import MarkerArray

class Ring:
    def __init__(self,color,pose):
        self.color = color
        self.pose = pose

class Cylinder:
    def __init__(self,color,pose):
        self.color = color
        self.pose = pose

class Parking:
    def __init__(self,pose):
        self.pose = pose


class MarkerManager:
    def __init__(self):
        #position of latest face
        self.face_to_approach = None
        self.parking_to_approach = None

        self.ring_colors = []
        self.cylinder_colors = []
        #arrays of rings, cyliders and parking
        self.rings = []
        self.cylinders = []
        
        self.sm = SpeakingManager()

        self.face_marker_sub = rospy.Subscriber(
        "face_markers", MarkerArray, self.face_marker_callback, queue_size=10)
        self.ring_marker_sub = rospy.Subscriber(
            "ring_markers", MarkerArray, self.ring_marker_callback, queue_size=10)
        self.parking_marker_sub = rospy.Subscriber(
            "parking_markers", MarkerArray, self.parking_marker_callback, queue_size=10)
        self.cylinder_marker_sub = rospy.Subscriber(
            "detected_cylinders", MarkerArray, self.cylinder_marker_callback, queue_size=10)

    def get_color_from_rgba(self,rgba):
        """
        Returns the vector with marker color according to string

        colors = ["yellow", "green", "black", "blue", "red"]
        """
        #print(rgba)
        res = "white"
        if rgba == ColorRGBA(1, 0, 0, 1):
            res = "red"
        if rgba == ColorRGBA(0, 1, 0, 1):
            res = "green"
        if rgba == ColorRGBA(0, 0, 1, 1):
            res = "blue"
        if rgba == ColorRGBA(0, 0, 0, 1):
            res = "black"
        if rgba == ColorRGBA(255, 165, 0, 1):
            res = "yellow"

        return res
    
    def get_color_from_rgba_cylinder(self,rgba):
        """
        Returns the vector with marker color according to string

        colors = ["yellow", "green", "black", "blue", "red"]
        """
        r, g, b, a = rgba
        rgba = ColorRGBA(r,g,b,a)
        res = "white"
        if rgba == ColorRGBA(1, 0, 0, 1):
            res = "red"
        if rgba == ColorRGBA(0, 1, 0, 1):
            res = "green"
        if rgba == ColorRGBA(0, 0, 1, 1):
            res = "blue"
        if rgba == ColorRGBA(0, 0, 0, 1):
            res = "black"
        if rgba == ColorRGBA(255, 165, 0, 1):
            res = "yellow"

        return res

    def face_marker_callback(self,data_face):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)

        latest_marker = data_face.markers[-1]
        latest_marker_pose = latest_marker.pose

        face_x = latest_marker_pose.position.x
        face_y = latest_marker_pose.position.y
        face_z = latest_marker_pose.position.z

        #rospy.loginfo(f"Face to approach: x: {face_x}, y: {face_y}, z: {face_z}")
        self.face_to_approach = latest_marker_pose


    def ring_marker_callback(self,data):
        latest_ring = data.markers[-1]
        latest_ring_pose = latest_ring.pose
        latest_ring_color = latest_ring.color

        ring_color = self.get_color_from_rgba(latest_ring_color)

        self.sm.say_ring_color(ring_color)
        self.ring_colors.append(ring_color)

        ring = Ring(ring_color,latest_ring_pose)
        # print("New ring added:")
        # print(f"Ring color: {ring.color} Ring pose: {ring.pose}")
        self.rings.append(ring)


    def parking_marker_callback(self,data):
        parking = data.markers[-1]
        latest_parking_pose = parking.pose

        self.parking_to_approach = latest_parking_pose

    def cylinder_marker_callback(self,data):
        latest_cylinder = data.markers[-1]
        latest_cylinder_pose = latest_cylinder.pose
        latest_cylinder_color = latest_cylinder.color

        cylinder_color = self.get_color_from_rgba(latest_cylinder_color)
        self.sm.say_cylinder_color(cylinder_color)
        self.cylinder_colors.append(cylinder_color)

        cylinder = Cylinder(cylinder_color,latest_cylinder_pose)
        self.cylinders.append(cylinder)