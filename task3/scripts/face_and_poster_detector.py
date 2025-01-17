#!/usr/bin/python3

import sys
import rospy
import cv2
import numpy as np
import tf2_geometry_msgs
import tf2_ros

from os.path import dirname, join

# import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import time
import pyocr
import pyocr.builders
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PIL import Image as PILImage
import re
from task3.msg import PosterMessage

class Poster:
    def __init__(self, color, image, prize, pose):
        self.color = color
        self.pose = None
        self.image = image
        self.prize = prize


class Cluster:
    def __init__(self, first_point):
        self.points = [first_point]
        self.centroid = np.array([first_point.x, first_point.y, first_point.z])
        self.n = 1
        self.show_marker = False

    def compute_centroid(self):
        pts = np.array([(p.x, p.y, p.z) for p in self.points])
        self.centroid = np.mean(pts, axis=0)

    def add_new_point(self, point):
        self.n += 1
        if self.n > 3:
            return
        rospy.loginfo(f"Points in the cluster: {self.n}")
        self.points.append(point)
        self.compute_centroid()
        if self.n == 3:
            self.show_marker = True


class face_localizer:
    def __init__(self):
        rospy.init_node('face_localizer', anonymous=True)

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # The function for performin HOG face detection
        # self.face_detector = dlib.get_frontal_face_detector()
        protoPath = join(dirname(__file__), "deploy.prototxt.txt")
        modelPath = join(dirname(__file__),
                         "res10_300x300_ssd_iter_140000.caffemodel")

        self.face_net = cv2.dnn.readNetFromCaffe(protoPath, modelPath)

        # A help variable for holding the dimensions of the image
        self.dims = (0, 0, 0)

        # Marker array object used for showing markers in Rviz
        self.marker_array = MarkerArray()
        self.poster_marker_array = MarkerArray()
        self.marker_num = 1
        self.clusters = []

        # Subscribe to the image and/or depth topic
        # self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        # self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)

        # Publiser for the visualization markers
        self.markers_pub = rospy.Publisher(
            'face_markers', MarkerArray, queue_size=1000)
        #self.markers_pub.publish(MarkerArray())

        # Publiser for the posters 
        self.poster_pub = rospy.Publisher(
            'posters', PosterMessage, queue_size=1000)
        self.poster_markers_pub = rospy.Publisher(
            'poster_markers', MarkerArray, queue_size=1000)



        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        # Initialize the OCR tool
        self.ocr_tool = pyocr.get_available_tools()[0]
        self.ocr_lang = 'eng'

        # Initialize the CvBridge
        self.bridge = CvBridge()

    def euclidian_distance(self, point1, point2):
        """Function to calculate euclidean distance between two points."""

        return np.sqrt((point1.x - point2[0])**2 + (point1.y - point2[1])**2 + (point1.z - point2[2])**2)

    # def find_closest_cluster(self, point, clusters, threshold):
    #     """Function to find the closest cluster to a point."""
    #     n = (len(clusters))
    #     if n > 0:
    #         for i in range(len(clusters)):
    #             sum_position = [0, 0, 0]
    #             for posit in clusters[i]:
    #                 sum_position[0] += posit.x
    #                 sum_position[1] += posit.y
    #                 sum_position[2] += posit.z

    #         # Calculate the mean position and orientation
    #         mean_position = [sum_position[0] / len(clusters[i]), sum_position[1] / len(
    #             clusters[i]), sum_position[2] / len(clusters[i])]
    #         distance = self.euclidean_distance(point, mean_position)
    #         if distance < threshold:
    #             return i
    #     return None

    def add_point_to_cluster(self, point, clusters, threshold):
        """Function to add a point to a cluster if it is close enough."""
        # closest_cluster = self.find_closest_cluster(point, clusters, threshold)

        # if closest_cluster is not None:
        #     if len(clusters[closest_cluster]) < 5:
        #         clusters[closest_cluster].append(point)
        # else:
        #     clusters.append([point])
        closest_cluster = None
        minDist = float("inf")

        for cluster in self.clusters:
            dist = self.euclidian_distance(point, cluster.centroid)
            if dist > threshold:
                continue
            if dist < minDist:
                minDist = dist
                closest_cluster = cluster

        if closest_cluster == None:
            closest_cluster = Cluster(point)
            self.clusters.append(closest_cluster)
        else:
            closest_cluster.add_new_point(point)


    def get_pose(self, coords, dist, stamp):
        # Calculate the position of the detected face

        k_f = 554  # kinect focal length in pixels

        x1, x2, y1, y2 = coords

        face_x = self.dims[1] / 2 - (x1+x2)/2.
        face_y = self.dims[0] / 2 - (y1+y2)/2.

        angle_to_target = np.arctan2(face_x, k_f)

        # Get the angles in the base_link relative coordinate system
        x, y = dist*np.cos(angle_to_target), dist*np.sin(angle_to_target)

        # Define a stamped message for transformation - directly in "base_link"
        # point_s = PointStamped()
        # point_s.point.x = x
        # point_s.point.y = y
        # point_s.point.z = 0.3
        # point_s.header.frame_id = "base_link"
        # point_s.header.stamp = rospy.Time(0)

        # Define a stamped message for transformation - in the "camera rgb frame"
        point_s = PointStamped()
        point_s.point.x = -y
        point_s.point.y = 0
        point_s.point.z = x
        point_s.header.frame_id = "camera_rgb_optical_frame"
        point_s.header.stamp = stamp

        # Get the point in the "map" coordinate system
        try:
            point_world = self.tf_buf.transform(point_s, "map")

            # Create a Pose object with the same position
            pose = Pose()
            pose.position.x = point_world.point.x
            pose.position.y = point_world.point.y
            pose.position.z = point_world.point.z
        except Exception as e:
            print(e)
            pose = None

        return pose

    def check_poster(self, image, face_image, pose):
        cv_image = image

        poster = None

        # Convert the CV image to a PIL image
        pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        
        # Perform OCR on the PIL image
        text = self.ocr_tool.image_to_string(pil_image, lang=self.ocr_lang, builder=pyocr.builders.TextBuilder()).lower()
        
        # Print the extracted text
        #rospy.loginfo('Extracted text: {}'.format(text))
        colors = ["blue", "green", "black", "red"]

        if "wan" in text:
            print("This is a poster.")
            is_poster = True
            poster_ring_color = None
            poster_prize = -1
            words = ["blue", "green", "black", "red"]
            for color in colors:
                if color in text:
                    print(f"Ring color is {color}")
                    poster_ring_color = color
                    break
                else:
                    poster_ring_color = "unknown"
            match = re.search(r'\d+', text)

            if match:
                number = int(match.group()) * 1000
                poster_prize = number
                print("Prize:", number)
            else:
                print("No number found in text.")
                poster_prize = -1

            poster = Poster(poster_ring_color, face_image, poster_prize, pose)
            
        else:
            print("This is a face.")



        # cv2.imshow('Image', cv_image)
        # cv2.waitKey(0)
        
        return poster

    def find_faces(self):
        print('I got a new image!')

        # Get the next rgb and depth images that are posted from the camera
        try:

            rgb_image_message = rospy.wait_for_message(
                "/camera/rgb/image_raw", Image)
   
        except Exception as e:
            print(e)
            return 0

        try:
            
            depth_image_message = rospy.wait_for_message(
                "/camera/depth/image_raw", Image)
        except Exception as e:
            print(e)
            return 0


        # Convert the images into a OpenCV (numpy) format

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_message, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            depth_image = self.bridge.imgmsg_to_cv2(
                depth_image_message, "32FC1")
        except CvBridgeError as e:
            print(e)

        # Set the dimensions of the image
        self.dims = rgb_image.shape
        h = self.dims[0]
        w = self.dims[1]

        # Tranform image to gayscale
        # gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

        # Do histogram equlization
        # img = cv2.equalizeHist(gray)

        # Detect the faces in the image
        # face_rectangles = self.face_detector(rgb_image, 0)
        blob = cv2.dnn.blobFromImage(cv2.resize(
            rgb_image, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0))
        self.face_net.setInput(blob)
        face_detections = self.face_net.forward()

        for i in range(0, face_detections.shape[2]):
            confidence = face_detections[0, 0, i, 2]
            if confidence > 0.5:
                box = face_detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                box = box.astype('int')
                x1, y1, x2, y2 = box[0], box[1], box[2], box[3]

                # Extract region containing face
                face_region = rgb_image[y1:y2, x1:x2]

                # Visualize the extracted face
                #cv2.imshow("ImWindow", face_region)
                #cv2.waitKey(1)

                # Find the distance to the detected face
                face_distance = float(np.nanmean(depth_image[y1:y2, x1:x2]))
                
                
                if np.isnan(face_distance) or face_distance > 1.8:
                    #print('Distance to face (discarded)', face_distance)
                    continue

                print('Distance to face', face_distance)

                # Get the time that the depth image was recieved
                depth_time = depth_image_message.header.stamp

                # Find the location of the detected face
                pose = self.get_pose(
                    (x1, x2, y1, y2), face_distance, depth_time)

                if pose is not None and pose.position.x is not None and pose.position.y is not None:
                    self.add_point_to_cluster(pose.position, self.clusters, 1)
                    
                    publish = False
                    # Create a marker used for visualization
                    for clust in self.clusters:
                        if not clust.show_marker:
                            continue
                        self.marker_num += 1
                        # sum_position = [0, 0, 0]
                        # for posit in clu:
                        #     sum_position[0] += posit.x
                        #     sum_position[1] += posit.y
                        #     sum_position[2] += posit.z

                        # Calculate the mean position and orientation
                        # mean_position = [
                        #     sum_position[0] / len(clu), sum_position[1] / len(clu), sum_position[2] / len(clu)]
                        # pose.position.x = mean_position[0]
                        # pose.position.y = mean_position[1]
                        # pose.position.z = mean_position[2]
                        pose = Pose()
                        pose.position.x = clust.centroid[0]
                        pose.position.y = clust.centroid[1]
                        pose.position.z = clust.centroid[2]

                        marker = Marker()
                        marker.header.stamp = rospy.Time(0)
                        marker.header.frame_id = 'map'
                        marker.pose = pose
                        marker.type = Marker.CUBE
                        marker.action = Marker.ADD
                        marker.frame_locked = False
                        marker.lifetime = rospy.Duration()
                        marker.id = self.marker_num
                        marker.scale = Vector3(0.1, 0.1, 0.1)
                        marker.color = ColorRGBA(0, 1, 0, 1)
                        # distances = np.array([])

                        potential_poster = self.check_poster(rgb_image, face_region, pose)
                        if potential_poster != None:
                            print("new poster marker appended")
                            marker.color = ColorRGBA(1, 1, 1, 1)
                            marker.scale = Vector3(0.2, 0.2, 0.2)

                            img_msg = self.bridge.cv2_to_imgmsg(potential_poster.image, encoding='bgr8')
                            msg = PosterMessage()
                            msg.image = img_msg
                            msg.color = potential_poster.color
                            msg.prize = potential_poster.prize
                            msg.header.stamp = rospy.Time(0)
                            self.poster_pub.publish(msg)
                            self.poster_marker_array.markers.append(marker)
                        else:
                            print("new face marker appended")
                            self.marker_array.markers.append(marker)
                        #cv2.imshow("ImWindow", face_region)
                        #cv2.waitKey(1)
                        
                        clust.show_marker = False
                        publish = True
                    if publish and potential_poster == None:
                        self.markers_pub.publish(self.marker_array)
                    elif publish:
                        self.poster_markers_pub.publish(self.poster_marker_array)



def main():

    time.sleep(2)
    face_finder = face_localizer()
    rate = rospy.Rate(4)
    
    while not rospy.is_shutdown():
        face_finder.find_faces()
        rate.sleep()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
