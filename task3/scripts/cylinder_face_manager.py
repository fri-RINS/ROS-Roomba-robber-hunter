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
from face_compare import ImageCompareManager
from geometry_msgs.msg import Twist
from move_arm import Arm_Mover
class CylinderFaceManager:
    def __init__(self):
        

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()
        self.am = Arm_Mover()
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
        self.marker_num = 1
        self.clusters = []
        self.cmd_vel_pub = rospy.Publisher(
        '/cmd_vel_mux/input/teleop', Twist, queue_size=100)
        self.face_found = False

        # Subscribe to the image and/or depth topic
        # self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        # self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)

        # Publiser for the visualization markers
        self.markers_pub = rospy.Publisher(
            'cylinder_face_markers', MarkerArray, queue_size=1000)
        #self.markers_pub.publish(MarkerArray())
        self.ready = False

    # def rotate(self):
    #     # Create a Twist message
    #     twist_cmd = Twist()
    #     twist_cmd.angular.z = 0.1  # Set angular velocity for left rotation

    #     # Create a publisher to publish velocity commands
    #     pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    #     # Publish the velocity command repeatedly for a short duration
    #     rate = rospy.Rate(10)  # 10 Hz
    #     duration = rospy.Duration(1.0)  # Rotate for 1 second
    #     start_time = rospy.Time.now()

    #     while rospy.Time.now() - start_time < duration:
    #         pub.publish(twist_cmd)
    #         rate.sleep()

    #     # Stop the rotation by publishing zero velocity
    #     twist_cmd.angular.z = 0.0
    #     pub.publish(twist_cmd)

    def euclidian_distance(self, point1, point2):
        """Function to calculate euclidean distance between two points."""

        return np.sqrt((point1.x - point2[0])**2 + (point1.y - point2[1])**2 + (point1.z - point2[2])**2)
    
    def rotate_left(self):
        twist_msg = Twist()
        twist_msg.angular.z = 0.5
        self.cmd_vel_pub.publish(twist_msg)
        rospy.sleep(1)
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)  

    def find_faces(self):
        self.face_found = False
        counter = 0
        while self.face_found is False:
            if counter > 15:
                return None
            counter +=1
            print('I got a new cylinder face image!')

            # Get the next rgb and depth images that are posted from the camera
            try:

                rgb_image_message = rospy.wait_for_message(
                    "/arm_camera/rgb/image_raw", Image)
    
            except Exception as e:
                print(e)
                return 0


            # Convert the images into a OpenCV (numpy) format

            try:
                rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_message, "bgr8")
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
                    print(face_region)
                    #cv2.imwrite("src/hw3/task3/img_test/image_cyl_whole.jpg", rgb_image)

                    if face_region.shape[1] > 0 and face_region.shape[0]>0:
                        print((face_region.shape))
                        cv2.imshow("ImWindow", face_region)
                        cv2.waitKey(0)
                        cv2.destroyAllWindows()
                        self.face_found = True
                        face_region = cv2.cvtColor(face_region, cv2.COLOR_BGR2RGB)
                        face_region = cv2.cvtColor(face_region, cv2.COLOR_RGB2BGR)
                        return face_region  
                    
            self.rotate_left()               

def main():
    rospy.init_node('cylinder_face_manager', anonymous=True)
    time.sleep(2)
    cylinder_face_manager = CylinderFaceManager()
    cylinder_face_manager.am.extend_to_face()
    rate = rospy.Rate(2)
    face_region = None
    while face_region is None:
        face_region = cylinder_face_manager.find_faces()
        rate.sleep()
    #print(face_region)

    cylinder_face_manager.am.retract_camera()
    

    icm = ImageCompareManager()
    image_path1 = 'src/hw3/task3/img_test/image1.jpg'
    image_path2 = 'src/hw3/task3/img_test/image2.jpg'
    image1 = cv2.imread(image_path1)
    image2 = cv2.imread(image_path2)

    conf1 = icm.compare_faces(face_region, image1)
    conf2 = icm.compare_faces(face_region, image2)

    # cv2.imshow("ImWindow", face_region)
    # cv2.waitKey(0)
    # rospy.sleep(10)

    #cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
