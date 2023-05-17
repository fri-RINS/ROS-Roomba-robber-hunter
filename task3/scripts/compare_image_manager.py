#!/usr/bin/python3

import rospy
import cv2
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
import numpy as np
import time
from sklearn.metrics.pairwise import cosine_similarity
import dlib
from cv_bridge import CvBridge



class ImageCompareManager():
    def __init__(self, cylinder_image):
        self.cylinder_image = cylinder_image   
        self.poster_image1 = cv2.imread("src/hw3/task3/img_test/image1.jpg")
        self.poster_image2 = cv2.imread("src/hw3/task3/img_test/image2.jpg")

        self.bridge = CvBridge()
        self.face_detector = dlib.get_frontal_face_detector()
        self.face_reidentifier = dlib.face_recognition_model_v1('src/hw3/task3/img_test/dlib_face_recognition_resnet_model_v1.dat')
        face_recognizer = cv2.face.LBPHFaceRecognizer_create()
        face_recognizer.read('path/to/face_recognition_model.xml')

        
    def compute_face_descriptor(self, cv_image):
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        faces = self.face_detector(gray_image)

        if len(faces) < 1:
            rospy.logwarn("No faces detected.")
            return

        # Compute the face descriptor for the first detected face
        landmarks = self.face_reidentifier(gray_image, faces[0])
        face_descriptor = self.face_reidentifier.compute_face_descriptor(gray_image, landmarks)

        
        return face_descriptor

    def embed_face(self, image):
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        landmarks = self.face_reidentifier(gray_image)
        face_descriptor = self.face_reidentifier.compute_face_descriptor(image, landmarks)
        return np.array(face_descriptor)

    def compare_images(self, image1, image2):

        
        # face_descriptor1 = self.compute_face_descriptor(image1)
        # face_descriptor2 = self.compute_face_descriptor(image2)

        # # match = dlib.face_distance(face_descriptor1, face_descriptor2) < 0.6
        # match = (np.linalg.norm(np.array(face_descriptor1) - np.array(face_descriptor2), axis=1)) < 0.6

        # embedding_1 = self.embed_face(image1)
        # embedding_2 = self.embed_face(image2)

        # similarity_score = cosine_similarity(embedding_1, embedding_2)

        # match = similarity_score > 0.6

        # # Check if there is a match on either image
        # if match:
        #     rospy.loginfo("Same face detected!")
        # else:
        #     rospy.loginfo("Different faces.")

        # Load reference face images
        reference_image1 = cv2.imread('path/to/reference_image1.jpg', cv2.IMREAD_GRAYSCALE)
        reference_image2 = cv2.imread('path/to/reference_image2.jpg', cv2.IMREAD_GRAYSCALE)

        # Perform face recognition on reference images
        reference_label1, _ = face_recognizer.predict(reference_image1)
        reference_label2, _ = face_recognizer.predict(reference_image2)

        # Load test image
        test_image = cv2.imread('path/to/test_image.jpg', cv2.IMREAD_GRAYSCALE)

        # Perform face recognition on the test image
        test_label, _ = face_recognizer.predict(test_image)

        # Compare labels of reference and test images
        if test_label == reference_label1 or test_label == reference_label2:
            print("Same person")
        else:
            print("Different person")


    def compare_images1(self, image1, image2):
        # Load the images

        # Convert images to grayscale
        #gray1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
        #gray2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)

        # # # Resize the faces to a common size for better comparison
        face1 = cv2.resize(image1, (150, 150))
        face2 = cv2.resize(image2, (150, 150))


        # # Normalize pixel values
        face1 = face1.astype("float32") / 255.0
        face2 = face2.astype("float32") / 255.0

        # Extract face embeddings for both images
        embedding1 = np.array(self.extract_face_embeddings(face1)).reshape(-1, 1)
        embedding2 = np.array(self.extract_face_embeddings(face2)).reshape(-1, 1)
        
        similarity_score = cosine_similarity(embedding1, embedding2)

        # Compute average similarity score
        average_similarity_score = np.mean(similarity_score)
 
        print("SIMILARITY:", average_similarity_score)

        # Define a threshold to determine if the images represent the same person
        threshold = 0.7

        # Compare the similarity score with the threshold
        if average_similarity_score > threshold:
            print("The images represent the same person.")
            return True
        else:
            print("The images represent different persons.")
            return False

def main():
    rospy.init_node('compare_image_node', anonymous=True)
    time.sleep(2)

    cylinder_image_path = "src/hw3/task3/img_test/image_cylinder.jpg"
    cylinder_image = cv2.imread(cylinder_image_path)
    
    icm = ImageCompareManager(cylinder_image)
    # Call the function to compare the images
    icm.compare_images(cylinder_image, icm.poster_image1)
    icm.compare_images(cylinder_image, icm.poster_image2)


if __name__ == '__main__':
    main()
       
