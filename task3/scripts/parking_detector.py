#!/usr/bin/python3

import sys
import rospy
import cv2
import numpy as np
import tf2_geometry_msgs
import tf2_ros
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import math
import time
from std_msgs.msg import String


can_start = False
start_park_detection_sub = None

class ParkingDetector:
    def __init__(self):

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # A help variable for holding the dimensions of the image
        self.dims = (0, 0, 0)

        # Marker array object used for visualizations
        self.marker_array = MarkerArray()
        self.marker_num = 1

        self.object_num_threshold = 1
        #self.object_proximity_threshold = 0.5
        self.object_proximity_threshold = 0.3
        # only detect object if distance to it smaller than
        # best for accurate positioning:
        # self.max_detection_distance = 2.0
        #self.max_detection_distance = 10.0
        self.max_detection_distance = 1.9

        # poses of potential objects (1 pose per group)
        self.potential_object_poses = []
        # number of votes for pose at i-th position in self.potential_object_poses
        self.potential_object_votes = []
        # tuples (xr, yr, rr_x, rr_y, rr_z, rr_w) where the robot was and how it was rotated when potential face at i-th position in self.potential_object_poses was detected
        self.robot_at_detection_coords = []

        # False if object pose ton yet sent
        self.potencial_objects_already_sent = []

        # to store colors of objects at corresponding positions
        self.object_colors = []

        # images of objects at corresponding positions
        self.object_images = []

        # number of published objects
        self.number_of_published_objects = 0
        self.num_all_objects = 10
        self.found_all_objects = False

        # for publishing face markers on map
        self.marker_array = MarkerArray()
        self.marker_num = 1
        self.markers_pub = rospy.Publisher('parking_markers', MarkerArray, queue_size=1000)

        # Subscribe to the image and/or depth topic
        self.image_sub = rospy.Subscriber("/arm_camera/rgb/image_raw", Image, self.image_callback)
        # self.depth_sub = rospy.Subscriber("/camera/depth_registered/image_raw", Image, self.depth_callback)


        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

    def dist_euclidean(self,x1, y1, x2, y2):
        #rospy.loginfo("Distance called with: (%s, %s, %s, %s)" % (str(x1), str(y1), str(x2), str(y2)))
        return math.sqrt((x1 - x2) **2 + (y1 - y2)**2)


    def publish_pose(self, pose, marker_color):
        # Create a marker used for visualization
        self.marker_num += 1
        marker = Marker()
        marker.header.stamp = rospy.Time(0)
        marker.header.frame_id = 'map'
        marker.pose = pose
        #marker.type = Marker.CUBE
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.lifetime = rospy.Duration.from_sec(500)
        marker.id = self.marker_num
        marker.scale = Vector3(0.1, 0.1, 0.1)
        #marker.color = ColorRGBA(0, 1, 0, 1)
        marker.color = marker_color
        self.marker_array.markers.append(marker)

        self.markers_pub.publish(self.marker_array)

    def publish_unique_object(self, object_pose, robot_detection_pose):
        print("Publishing new unique object")
        self.publish_pose(object_pose, ColorRGBA(0,0,0,1))

        # self.publish_greet_instructions(object_pose, robot_detection_pose, object_color)

        # increase the number of objects
        self.number_of_published_objects = self.number_of_published_objects + 1

        # stop execution if all objects are found
        if self.number_of_published_objects >= self.num_all_objects:
            rospy.loginfo("Exiting face search.")
            #sys.exit(0)
            #exit()
            self.found_all_objects = True

    def new_potential_object_pose(self, object_image, object_pose, robot_pose=None):
        """
        TODO: currently robot_pose set to None for easier testing
        Goes through all already detected object positions and votes for the one that is closer than
        some threshold. Otherwise it creates a new cell and votes for it.

        It immediately publishes when enough detections are collected for certain object.
        """
        #rospy.loginfo("New potential object: (%s, %s)" % (str(object_pose.position.x), str(object_pose.position.y)))

        print(str(self.potential_object_votes))

        closest_not_found = True
        for i in range(len(self.potential_object_poses)):
            d = self.dist_euclidean(object_pose.position.x, object_pose.position.y, self.potential_object_poses[i].position.x, self.potential_object_poses[i].position.y)

            if d < self.object_proximity_threshold:
                voting_weight = self.potential_object_votes[i]

                self.potential_object_votes[i] = self.potential_object_votes[i] + 1
                self.robot_at_detection_coords[i] = robot_pose

                # use weighted average to compute updated object position
                self.potential_object_poses[i].position.x = (voting_weight * self.potential_object_poses[i].position.x + object_pose.position.x) / (voting_weight + 1)
                self.potential_object_poses[i].position.y = (voting_weight * self.potential_object_poses[i].position.y + object_pose.position.y) / (voting_weight + 1)
                self.potential_object_poses[i].position.z = (voting_weight * self.potential_object_poses[i].position.z + object_pose.position.z) / (voting_weight + 1)

                closest_not_found = False

                


                # we send the face immediately when enough detections are noted
                if (self.potential_object_votes[i] >= self.object_num_threshold) and (not self.potencial_objects_already_sent[i]):
                    self.publish_unique_object(self.potential_object_poses[i], self.robot_at_detection_coords[i])
                    self.potencial_objects_already_sent[i] = True

        # if no stored pose is close enough add new position
        if closest_not_found:
            #rospy.loginfo("Closest not found.")
            self.potential_object_poses.append(object_pose)
            self.potential_object_votes.append(1)
            self.robot_at_detection_coords.append(robot_pose)
            self.potencial_objects_already_sent.append(False)

            #for color
            self.object_images.append(object_image)

            """
            # get image color
            successful_service_call = False
            
            ###
            try:
                #img = self.bridge.cv2_to_imgmsg(object_image, encoding="passthrough")
                img = self.bridge.cv2_to_imgmsg(object_image, encoding="bgr8")
            except CvBridgeError as e:
                print(e)

            
            rospy.wait_for_service('detect_color')
            try:
                color_srv = rospy.ServiceProxy('detect_color', ColorSrv)
                resp_raw = color_srv(img)
                resp = resp_raw.color
                successful_service_call = True
            except rospy.ServiceException as e:
                print('Service call failed.')
            ###

            if successful_service_call:
                self.object_colors.append(resp)
            else:
                self.object_colors.append("unknown")
            """

            # write image to disk
            #cv2.imwrite("ring_" + str(len(self.potential_object_poses)) + ".jpg", object_image)

        #rospy.loginfo("New vote tally: %s" % str(self.potential_object_votes))

    def get_pose(self,e,dist):
        # Calculate the position of the detected ellipse

        k_f = 525 # kinect focal length in pixels

        elipse_x = self.dims[1] / 2 - e[0][0]
        elipse_y = self.dims[0] / 2 - e[0][1]

        angle_to_target = np.arctan2(elipse_x,k_f)

        # Get the angles in the base_link relative coordinate system
        x,y = dist*np.cos(angle_to_target), dist*np.sin(angle_to_target)

        ### Define a stamped message for transformation - directly in "base_frame"
        #point_s = PointStamped()
        #point_s.point.x = x
        #point_s.point.y = y
        #point_s.point.z = 0.3
        #point_s.header.frame_id = "base_link"
        #point_s.header.stamp = rospy.Time(0)

        # Define a stamped message for transformation - in the "camera rgb frame"
        point_s = PointStamped()
        point_s.point.x = -y
        point_s.point.y = 0
        point_s.point.z = x
        point_s.header.frame_id = "arm_camera_rgb_optical_frame"
        point_s.header.stamp = rospy.Time(0)

        # Get the point in the "map" coordinate system
        point_world = self.tf_buf.transform(point_s, "map")

        # Create a Pose object with the same position
        pose = Pose()
        pose.position.x = point_world.point.x
        pose.position.y = point_world.point.y
        pose.position.z = point_world.point.z

        return pose


    def image_callback(self,data):
        print('I got a new image!')

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Set the dimensions of the image
        self.dims = cv_image.shape

        # Tranform image to gayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Do histogram equlization
        img = cv2.equalizeHist(gray)

        # Binarize the image, there are different ways to do it
        #ret, thresh = cv2.threshold(img, 50, 255, 0)
        #ret, thresh = cv2.threshold(img, 70, 255, cv2.THRESH_BINARY)
        thresh = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 15, 25)

        # Extract contours
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Example how to draw the contours, only for visualization purposes
        cv2.drawContours(img, contours, -1, (255, 0, 0), 3)
        # cv2.imshow("Contour window",img)
        # cv2.waitKey(1)

        # Fit elipses to all extracted contours
        elps = []
        for cnt in contours:
            #     print cnt
            #     print cnt.shape
            if cnt.shape[0] >= 20:
                ellipse = cv2.fitEllipse(cnt)
                elps.append(ellipse)


        # Find two elipses with same centers
        candidates = []
        for n in range(len(elps)):
            for m in range(n + 1, len(elps)):
                e1 = elps[n]
                e2 = elps[m]
                dist = np.sqrt(((e1[0][0] - e2[0][0]) ** 2 + (e1[0][1] - e2[0][1]) ** 2))
                #             print dist
                if dist < 5:
                    candidates.append((e1,e2))

        print("Processing is done! found", len(candidates), "candidates for rings")

        try:
            depth_img = rospy.wait_for_message('/arm_camera/depth/image_raw', Image)
        except Exception as e:
            print(e)

        # Extract the depth from the depth image
        for c in candidates:

            # the centers of the ellipses
            e1 = c[0]
            e2 = c[1]

            # drawing the ellipses on the image
            cv2.ellipse(cv_image, e1, (0, 255, 0), 2)
            cv2.ellipse(cv_image, e2, (0, 255, 0), 2)

            size = (e1[1][0]+e1[1][1])/2
            center = (e1[0][1], e1[0][0])

            x1 = int(center[0] - size / 2)
            x2 = int(center[0] + size / 2)
            x_min = x1 if x1>0 else 0
            x_max = x2 if x2<cv_image.shape[0] else cv_image.shape[0]

            y1 = int(center[1] - size / 2)
            y2 = int(center[1] + size / 2)
            y_min = y1 if y1 > 0 else 0
            y_max = y2 if y2 < cv_image.shape[1] else cv_image.shape[1]

            # for outer elypse
            size_outer = (e2[1][0]+e2[1][1])/2
            center_outer = (e1[0][1], e1[0][0])

            x1_outer = int(center_outer[0] - size_outer / 2)
            x2_outer = int(center_outer[0] + size_outer / 2)
            x_min_outer = x1_outer if x1_outer>0 else 0
            x_max_outer = x2_outer if x2_outer<cv_image.shape[0] else cv_image.shape[0]

            y1_outer = int(center_outer[1] - size_outer / 2)
            y2_outer = int(center_outer[1] + size_outer / 2)
            y_min_outer = y1_outer if y1_outer > 0 else 0
            y_max_outer = y2_outer if y2_outer < cv_image.shape[1] else cv_image.shape[1]

            depth_image = self.bridge.imgmsg_to_cv2(depth_img, "16UC1")

            pose = self.get_pose(e1, float(np.mean(depth_image[x_min:x_max,y_min:y_max]))/1000.0)

            if pose is not None:
                #self.publish_pose(pose)
		       #print("pose is not NONE")
                # correctly deal with new potential ring
                object_image = cv_image[x_min_outer:x_max_outer, y_min_outer:y_max_outer]
                self.new_potential_object_pose(object_image, pose, robot_pose=None)



    def depth_callback(self,data):

        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError as e:
            print(e)

        # Do the necessairy conversion so we can visuzalize it in OpenCV
        image_1 = depth_image / 65536.0 * 255
        image_1 =image_1/np.max(image_1)*255

        image_viz = np.array(image_1, dtype= np.uint8)

        cv2.imshow("Depth window", image_viz)
        cv2.waitKey(1)


def main():

    rospy.init_node('parking_detector_node', anonymous=True)
    rate = rospy.Rate(1)


if __name__ == '__main__':
    main()