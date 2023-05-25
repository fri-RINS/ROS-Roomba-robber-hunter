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
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped



class ParkingDetector:
    def __init__(self):
        self.bridge = CvBridge()    # An object we use for converting images between ROS format and OpenCV format
        self.dims = (0, 0, 0)       # A help variable for holding the dimensions of the image
        

        # Marker array object used for visualizations
        self.marker_array = MarkerArray()
        self.marker_num = 3

        self.object_num_threshold = 1
        self.object_proximity_threshold = 0.3    # only detect object if distance to it smaller than

        self.potential_object_poses = []         # poses of potential objects (1 pose per group)
        self.potential_object_votes = []         # number of votes for pose at i-th position in self.potential_object_poses
        self.potencial_objects_already_sent = [] # False if object pose ton yet sent
        self.object_colors = []                  # to store colors of objects at corresponding positions
        self.object_images = []                  # images of objects at corresponding positions

        self.parking_found = False

        # for publishing face markers on map
        self.marker_array = MarkerArray()
        self.marker_num = 1
        self.markers_pub = rospy.Publisher('parking_markers', MarkerArray, queue_size=1000)

        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        self.cmd_vel_pub = rospy.Publisher(
        '/cmd_vel_mux/input/teleop', Twist, queue_size=100)
        self.odom_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.current_robot_pose_callback)

    def current_robot_pose_callback(self,data):
        self.current_robot_pose = data.pose.pose

    def dist_euclidean(self,x1, y1, x2, y2):
        return math.sqrt((x1 - x2) **2 + (y1 - y2)**2)


    def publish_marker(self, pose):
        # Create a marker used for visualization
        self.marker_num += 1
        marker = Marker()
        marker.header.stamp = rospy.Time(0)
        marker.header.frame_id = 'map'
        marker.pose = pose
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.lifetime = rospy.Duration.from_sec(500)
        marker.id = self.marker_num
        marker.scale = Vector3(0.1, 0.1, 0.1)
        marker.color = ColorRGBA(0, 0, 0, 1)
        self.marker_array.markers.append(marker)
        self.markers_pub.publish(self.marker_array)

        rospy.loginfo("New parking marker appended.")

        

    def new_potential_parking_pose(self, object_image, object_pose):
        
        #rospy.loginfo("New potential object: (%s, %s)" % (str(object_pose.position.x), str(object_pose.position.y)))

        print(str(self.potential_object_votes))

        closest_not_found = True
        for i in range(len(self.potential_object_poses)):
            d = self.dist_euclidean(object_pose.position.x, object_pose.position.y, self.potential_object_poses[i].position.x, self.potential_object_poses[i].position.y)

            if d < self.object_proximity_threshold:
                voting_weight = self.potential_object_votes[i]

                self.potential_object_votes[i] = self.potential_object_votes[i] + 1

                # use weighted average to compute updated object position
                self.potential_object_poses[i].position.x = (voting_weight * self.potential_object_poses[i].position.x + object_pose.position.x) / (voting_weight + 1)
                self.potential_object_poses[i].position.y = (voting_weight * self.potential_object_poses[i].position.y + object_pose.position.y) / (voting_weight + 1)
                self.potential_object_poses[i].position.z = (voting_weight * self.potential_object_poses[i].position.z + object_pose.position.z) / (voting_weight + 1)

                closest_not_found = False

                # we send the face immediately when enough detections are noted
                if (self.potential_object_votes[i] >= self.object_num_threshold) and (not self.potencial_objects_already_sent[i]):
                    self.potencial_objects_already_sent[i] = True
                    self.parking_pose = self.potential_object_poses[i]
                    self.parking_found = True
                    return

        # if no stored pose is close enough add new position
        if closest_not_found:
            #rospy.loginfo("Closest not found.")
            self.potential_object_poses.append(object_pose)
            self.potential_object_votes.append(1)
            self.potencial_objects_already_sent.append(False)

            #for color
            self.object_images.append(object_image)

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

        # Define a stamped message for transformation - in the "camera rgb frame"
        point_s = PointStamped()
        point_s.point.x = -y
        point_s.point.y = 0
        point_s.point.z = x
        point_s.header.frame_id = "arm_camera_rgb_optical_frame"
        point_s.header.stamp = rospy.Time(0)

        rate = rospy.Rate(1)
        rate.sleep()
        # Get the point in the "map" coordinate system
        point_world = self.tf_buf.transform(point_s, "map")

        # Create a Pose object with the same position
        pose = Pose()
        pose.position.x = point_world.point.x
        pose.position.y = point_world.point.y
        pose.position.z = point_world.point.z

        return pose

    def get_elipse(self, cv_image) -> tuple:
        self.dims = cv_image.shape
        cv_image = cv_image[:440,:] # remove the bottom part of the image so the robot is not in the way
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, thresh1 = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY_INV)
        edged = cv2.Canny(thresh1, 50, 150)
        
        contours, hierarchy = cv2.findContours(edged, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        #cv2.drawContours(cv_image, contours, -1, (255, 0, 0), 3)
        #cv2.imshow("Contour window",cv_image)
        #cv2.waitKey(1)

        elps = []
        candidates = []
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
                    #print(dist)
                    #if dist < 5:
                    #    candidates.append((e1,e2))
                    candidates.append((e1,e2))

                
        center2size = {}
        for e1, e2 in candidates:
            size = round((e1[1][0]+e1[1][1])/2)
            center = (round(e1[0][1]), round(e1[0][0]))

            if center2size.get(center) == None:
                center2size[center] = [size]
            else:
                center2size.get(center).append(size)
                
        rings = [cv_image[
             max(obj[0] - np.max(center2size.get(obj)) // 2, 0):
             min(obj[0] + np.max(center2size.get(obj)) // 2, cv_image.shape[0]),
             max(obj[1] - np.max(center2size.get(obj)) // 2, 0):
             min(obj[1] + np.max(center2size.get(obj)) // 2, cv_image.shape[1])]
         for obj in center2size]

        if len(rings) == 0:
            return None

        x_avg, y_avg = np.mean(list(center2size.keys()), axis=0)

        return x_avg, y_avg

    def check_image(self):
        print('I got a new potential parking image!')

        # Get the next rgb and depth images that are posted from the camera
        try:

            rgb_image_message = rospy.wait_for_message(
                "/arm_camera/rgb/image_raw", Image)

        except Exception as e:
            print(e)
            return 0


        # Convert the images into a OpenCV (numpy) format

        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_image_message, "bgr8")
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
        #cv2.drawContours(img, contours, -1, (255, 0, 0), 3)
        #cv2.imshow("Contour window",img)
        #cv2.waitKey(1)

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

        print("Processing is done! found", len(candidates), "candidates for parking")

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
                self.new_potential_parking_pose(object_image, pose)



    # def depth_callback(self,data):

    #     try:
    #         depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
    #     except CvBridgeError as e:
    #         print(e)

    #     # Do the necessairy conversion so we can visuzalize it in OpenCV
    #     image_1 = depth_image / 65536.0 * 255
    #     image_1 =image_1/np.max(image_1)*255

    #     image_viz = np.array(image_1, dtype= np.uint8)

    #     cv2.imshow("Depth window", image_viz)
    #     cv2.waitKey(1)

    def rotate(self):
        
        twist = Twist()
        twist.angular.z = 0.4  # Adjust angular velocity as needed

        self.cmd_vel_pub.publish(twist)

    def stop_rotating(self):
        twist = Twist()
        twist.angular.z = 0.0  # Adjust angular velocity as needed

        self.cmd_vel_pub.publish(twist)

    def find_rough_parking(self):
        print("Find rough parking...")
        rate = rospy.Rate(10)

        twist = Twist()
        twist.linear.x = -0.55  # Adjust angular velocity as needed
        for _ in range(9):
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        twist.linear.x = 0.0  # Adjust angular velocity as needed
        self.cmd_vel_pub.publish(twist)
        # Rotate and check image for parking until parking found
        while not self.parking_found:
            self.rotate()
            self.check_image()
            rate.sleep()        



        self.stop_rotating()
        self.publish_marker(self.parking_pose)
        print("Found parking pose:", self.parking_pose)
        return self.parking_pose

    def fine_parking(self):
        print("Fine parking...")
        print("potential object poses:", self.potential_object_poses)
        print("current robot pose:", self.current_robot_pose)

        point_s = PointStamped()
        """
        try:
            self.arm_mover.publish(self.extend)
            print('camera ready')
        except Exception as e:
            print(e)
        """

        rospy.sleep(1.0)
        prev_is_set = False

        cond = True
        st = 0
        y_is_set = False
        while cond and st < 2000:
            try:
                rgb_img = rospy.wait_for_message('/arm_camera/rgb/image_raw', Image)
                point_s.header.frame_id = "arm_camera_rgb_optical_frame"
                point_s.header.stamp = rgb_img.header.stamp
            except Exception as e:
                print(e)

            try:
                cv_image = self.bridge.imgmsg_to_cv2(rgb_img, "bgr8")
            except CvBridgeError as e:
                print(e)

            x_avg, y_avg = self.get_elipse(cv_image)
            if prev_is_set:
                diff_x = abs(prev_x -x_avg)
                diff_y = abs(prev_y -y_avg)
                diff = diff_x + diff_y
            else:
                diff = 0
            print(x_avg, y_avg)
            msg = Twist()
            print("diff",diff)
            if diff < 10:
                st +=1
            else:
                st = 0
            if(diff < 1000):
                if 280<= y_avg <= 340 and 600 >=x_avg >= 400:
                    cond = False
                    dist = (1 - (x_avg / 600)) / 1.5
                    msg.linear.x = dist
                    print("dist",dist)

                else:
                    if 260 <= y_avg < 360:
                        msg.angular.z = 0.0
                        if x_avg > 450:
                            msg.linear.x = 0.01
                        elif 500>x_avg > 400:
                            msg.linear.x = 0.03
                        elif x_avg <= 400:
                            msg.linear.x = 0.07

                    if y_avg >= 360:
                        msg.linear.x = 0.01
                        msg.angular.z = -0.05

                    elif y_avg <= 260:
                        msg.linear.x = 0.01
                        msg.angular.z = 0.05

            else:
                #cond = False
                msg.linear.x = -3

            self.cmd_vel_pub.publish(msg)
            prev_is_set = True
            prev_x = x_avg
            prev_y = y_avg
            


def main():

    rospy.init_node('parking_detector_node', anonymous=True)
    pd = ParkingDetector()
    rospy.rate(2)
    parking_pose = pd.find_rough_parking()
    #pd.publish_marker(parking_pose)
    pd.fine_parking()


if __name__ == '__main__':
    main()