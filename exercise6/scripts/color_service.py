#!/usr/bin/python3

# vsebuje funkcijo, ki vrne koordinate ciljev, ki jih je potrebno doseci, da pregledamo celoten prostor
# se bo uporabil v brain.py na zacetku, da se pridobijo cilji

import rospy
import cv2
import numpy as np
import math

# for map loading
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped, Point, PointStamped, Vector3, Pose

import tf2_geometry_msgs
import tf2_ros

# for images additionally
from matplotlib import pyplot as plt

from tf import transformations as t
from cv_bridge import CvBridge, CvBridgeError

from course_project.srv import ColorSrv

import argparse


class ColorDetector:
    def __init__(self):
        pass


    def get_color(self, imageR):

        image = cv2.cvtColor(imageR, cv2.COLOR_BGR2HSV)

        # define the list of boundaries
        # red, blue, yellow, gray
        boundariesBGR = [
            ([17, 15, 100], [50, 56, 200]),
            ([86, 31, 4], [220, 88, 50]),
            ([25, 146, 190], [62, 174, 250])
            #,([103, 86, 65], [145, 133, 128])
        ]
        #yellow, green, black, white, blue, red*2
        boundariesHSV = [
            ([24,80,20],[32,255,255]),
            ([36,25,25],[80,255,255]),
            ([0,0,0],[180,255,50]),
            #([0,0,168],[172,111,255]),
            ([88,120,25],[133,255,255]),
            ([0,100,20],[10,255,255]),
            ([160,100,20],[180,255,255])
        ]


        count = []
        # loop over the boundaries
        for (lower, upper) in boundariesHSV:
            # create NumPy arrays from the boundaries
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")
            # find the colors within the specified boundaries and apply
            # the mask
            mask = cv2.inRange(image, lower, upper)
            output = cv2.bitwise_and(image, image, mask = mask)
            
            count.append(np.sum(mask)/255)
            # show the images
            #outputrgb = cv2.cvtColor(output, cv2.COLOR_HSV2BGR)
            #cv2.imshow("images", np.hstack([imageR, outputrgb]))
            #cv2.waitKey(0)
            

        count[-2] += count[-1]
        count.pop()
        #print("yellow, green, black, white, blue, red")
        print("yellow, green, black, blue, red")
        print(  [float("{:.2f}".format(i/sum(count))) for i in count] )
        
        colors = ["yellow", "green", "black", "blue", "red"]

        color_str = colors[count.index(max(count))]

        print(color_str)

        return color_str

    def get_color_from_image(self, rgb_image):
        detected_color = "todo"

        try:
            #rgb_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
            detected_color = self.get_color(rgb_image)
        except CvBridgeError as e:
            print(e)

        return detected_color


class ColorService:
    def __init__(self):
        rospy.init_node('color_service_server')
        s = rospy.Service('detect_color', ColorSrv, self.handle_color_request)
        self.bridge = CvBridge()
        rospy.loginfo('ready to recognise color')

        self.color_detector = ColorDetector()

    def handle_color_request(self, req):
        # TODO: do color recognition on image
        rospy.loginfo('Received image.')

        detected_color = "todo"

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(req.img, "bgr8")
            detected_color = self.color_detector.get_color(rgb_image)
        except CvBridgeError as e:
            print(e)

        return detected_color


if __name__ == '__main__':
    sr = ColorService()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()
