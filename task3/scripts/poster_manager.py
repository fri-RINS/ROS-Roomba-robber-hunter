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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PIL import Image as PILImage
import re
from task3.msg import PosterMessage
from std_msgs.msg import String
import pytesseract
import pyocr
from PIL import Image as PILimage

can_detect = False
poster = None

class Poster:
    def __init__(self, color, image, prize, is_poster):
        self.color = color
        self.pose = None
        self.image = image
        self.prize = prize
        self.is_poster = is_poster


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

        # # Publiser for the visualization markers
        # self.markers_pub = rospy.Publisher(
        #     'face_markers', MarkerArray, queue_size=1000)
        #self.markers_pub.publish(MarkerArray())

        # Publiser for the posters 
        self.poster_pub = rospy.Publisher(
            'posters', PosterMessage, queue_size=1000)
        self.poster_markers_pub = rospy.Publisher(
            'poster_markers', MarkerArray, queue_size=1000)

        # Initialize the CvBridge
        self.bridge = CvBridge()

    def euclidian_distance(self, point1, point2):
        """Function to calculate euclidean distance between two points."""

        return np.sqrt((point1.x - point2[0])**2 + (point1.y - point2[1])**2 + (point1.z - point2[2])**2)

    def preprocess_image(self, image):
    # Apply image preprocessing techniques (resize, denoise, contrast enhancement, etc.)
    # Return the preprocessed image
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
        # Apply image preprocessing techniques (e.g., resizing, denoising, contrast enhancement)
        # Here are some examples:
        
        # Resize the image to a specific width and height
        resized = cv2.resize(gray, (1200, 1000))
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(resized, (5, 5), 0)
        
        # Apply adaptive thresholding to enhance contrast
        thresholded = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 4)
        
        # Apply morphological operations to remove noise and improve digit connectivity
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4, 2))
        opening = cv2.morphologyEx(thresholded, cv2.MORPH_OPEN, kernel, iterations=1)
    
        # Return the preprocessed image
        return opening


    def perform_ocr(self, image):
        # Perform OCR on the image using pytesseract
        ocr_text = pytesseract.image_to_string(image, lang='eng')

        return ocr_text
        
    def img_to_text(self, image):
        # preprocessed_image = self.preprocess_image(image)
        # cv2.imshow("ImWindow", preprocessed_image)
        # cv2.waitKey(0)

        # # Perform OCR on the image or ROI
        # text = self.perform_ocr(preprocessed_image)
        # print("OCR1", text)

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # cv2.imshow("ImWindow", gray)
        # cv2.waitKey(0)
        # Initialize OCR engine
        ocr_tool = pyocr.get_available_tools()[0]  # Use the first available OCR engine
        ocr_lang = 'eng'  # Specify the language for OCR
        # Convert the grayscale image to RGB PIL image
        pil_image = PILimage.fromarray(gray)

        # Perform OCR on the image
        text = ocr_tool.image_to_string(pil_image, lang=ocr_lang)

        #print(text)


        return text

    def check_strings_in_text(self, strings, text):
        for string in strings:
            if string in text:
                return True
        return False


    def check_poster(self, image, face_image, pose):
        
        poster = None
        text = self.img_to_text(image).lower()
    
        # Print the extracted text
        #print('Extracted text: {}'.format(text))
        colors = ["blue", "green", "black", "red"]
        strings = ["blue", "green", "black", "red", "wan", "ted", "btc", "bic"]

        if self.check_strings_in_text(strings, text):
            #print("This is a poster.")
            is_poster = True
            poster_ring_color = None
            poster_prize = -1
            words = ["blue", "green", "black", "red"]
            for color in colors:
                if color in text:
                    #print(f"Ring color is {color}")
                    poster_ring_color = color
                    break
                else:
                    poster_ring_color = "unknown"
            match = re.search(r'\d+', text)

            if match:
                number = int(match.group())
                if number < 1000 and number > 0:
                    poster_prize = number * 1000
                elif number > 0:
                    poster_prize = number
                #print("Prize:", number)
            else:
                #print("No number found in text.")
                poster_prize = -1

            poster = Poster(poster_ring_color, face_image, poster_prize, True)
        
            
        #cv2.imshow('Image', cv_image)
        #cv2.waitKey(0)
        return poster

    def find_poster(self):
        poster = Poster(None, None, None, False)
        for i in range(3):
            potential_poster = self.detect_poster()
            if potential_poster != None:
                poster.image = potential_poster.image
                poster.is_poster = True
                if potential_poster.color != "unknown":
                    poster.color = potential_poster.color
                if potential_poster.prize != -1:
                    poster.prize = potential_poster.prize
                if poster.prize is not None and poster.color is not None:
                    break
                
        if poster.is_poster == True:
            print(f"This is a POSTER. Color: {poster.color} Prize:{poster.prize}")
            if poster.color is None:
                poster.color = "blue"
                print("Poster color set to blue:(")
            self.publish_marker(self.pose)
            print("new poster marker appended")
            return poster
        else:
            print("This is a face.")
            return None

    def publish_marker(self, pose):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.stamp = rospy.Time(0)
        marker.header.frame_id = 'map'
        marker.pose = self.pose
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.lifetime = rospy.Duration()
        marker.id = self.marker_num
        marker.color = ColorRGBA(1, 1, 1, 1)
        marker.scale = Vector3(0.2, 0.2, 0.2)
        self.marker_num += 1

        # self.poster_marker_array.markers.append(marker)
        marker_array.markers.append(marker)

        if marker.pose != None:
            #self.poster_markers_pub.publish(self.poster_marker_array)
            self.poster_markers_pub.publish(marker_array)

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
                
                # distances = np.array([])

                rgb_image_no_face = np.copy(rgb_image)
                rgb_image_no_face[y1:y2, :] = 255
                
                potential_poster = self.check_poster(rgb_image_no_face, face_region, self.pose)
                
                
                return potential_poster
   

def main():
    rospy.init_node('poster_detector', anonymous=True)
    find_posters = PosterDetector(None)
    rate = rospy.Rate(4)
    poster = find_posters.find_poster()

    #cv2.imwrite("src/hw3/task3/img_test/image2.jpg", poster.image)

    # while not rospy.is_shutdown():
    #     find_posters.detect_poster()
    #     rate.sleep()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
