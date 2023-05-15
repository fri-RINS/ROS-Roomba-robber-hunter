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
from std_msgs.msg import String
import pytesseract

can_detect = False
poster = None

class Poster:
    def __init__(self, color, image, prize):
        self.color = color
        self.pose = None
        self.image = image
        self.prize = prize


class PosterDetector:
    def __init__(self, pose):
        self.pose = pose
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

        # Initialize the OCR tool
        self.ocr_tool = pyocr.get_available_tools()[0]
        self.ocr_lang = 'eng'

        # Initialize the CvBridge
        self.bridge = CvBridge()

    def euclidian_distance(self, point1, point2):
        """Function to calculate euclidean distance between two points."""

        return np.sqrt((point1.x - point2[0])**2 + (point1.y - point2[1])**2 + (point1.z - point2[2])**2)


    def img_to_text(self, image):
        img = image
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
 
        # Performing OTSU threshold
        ret, thresh1 = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU | cv2.THRESH_BINARY_INV)
        
        # Specify structure shape and kernel size.
        # Kernel size increases or decreases the area
        # of the rectangle to be detected.
        # A smaller value like (10, 10) will detect
        # each word instead of a sentence.
        rect_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (18, 18))
        
        # Applying dilation on the threshold image
        dilation = cv2.dilate(thresh1, rect_kernel, iterations = 1)
        
        # Finding contours
        contours, hierarchy = cv2.findContours(dilation, cv2.RETR_EXTERNAL,
                                                        cv2.CHAIN_APPROX_NONE)
        
        # Creating a copy of image
        im2 = img.copy()
        
        # A text file is created and flushed
        poster_text = ""
        
        # Looping through the identified contours
        # Then rectangular part is cropped and passed on
        # to pytesseract for extracting text from it
        # Extracted text is then written into the text file
        # for cnt in contours:
        #     x, y, w, h = cv2.boundingRect(cnt)
            
        #     # Drawing a rectangle on copied image
        #     rect = cv2.rectangle(im2, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
        #     # Cropping the text block for giving input to OCR
        #     cropped = im2[y:y + h, x:x + w]
            
        # Apply OCR on the cropped image
        text = pytesseract.image_to_string(im2)
        
        # Appending the text into file
        poster_text += text
        #print(poster_text)

        # cv2.imshow("ImWindow", im2)
        # cv2.waitKey(1)

        return text

    def check_poster(self, image, face_image, pose):
        
        poster = None
        text = self.img_to_text(image).lower()
    
        # Print the extracted text
        print('Extracted text: {}'.format(text))
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

            poster = Poster(poster_ring_color, face_image, poster_prize)
            
        else:
            print("This is a face.")
            
        #cv2.imshow('Image', cv_image)
        #cv2.waitKey(0)
        return poster


    def detect_poster(self):
        print('I got a new potential poster!')

        # Get the next rgb and depth images that are posted from the camera
        try:

            rgb_image_message = rospy.wait_for_message(
                "/camera/rgb/image_raw", Image)
   
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
                #cv2.imshow("ImWindow", face_region)
                #cv2.waitKey(1)

                publish = False
                marker = Marker()
                marker.header.stamp = rospy.Time(0)
                marker.header.frame_id = 'map'
                marker.pose = self.pose
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.frame_locked = False
                marker.lifetime = rospy.Duration.from_sec(300)
                marker.id = self.marker_num
                marker.scale = Vector3(0.1, 0.1, 0.1)
                marker.color = ColorRGBA(0, 1, 0, 1)
                # distances = np.array([])

                rgb_image[y1:y2, x1:x2, :] = 0
                potential_poster = self.check_poster(rgb_image, face_region, self.pose)
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
                    publish = True
                if publish and potential_poster is not None:
                    self.poster_markers_pub.publish(self.poster_marker_array)

                return potential_poster
   

def main():
    rospy.init_node('poster_detector', anonymous=True)
    find_posters = PosterDetector(None)
    rate = rospy.Rate(4)
    find_posters.detect_poster()

    # while not rospy.is_shutdown():
    #     find_posters.detect_poster()
    #     rate.sleep()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
