#!/usr/bin/python3

import rospy
import cv2
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
import numpy as np
import time
from sklearn.metrics.pairwise import cosine_similarity
import dlib



class ImageCompareManager():
    def __init__(self, cylinder_image):
        self.cylinder_image = cylinder_image   
        self.poster_image1 = cv2.imread("src/hw3/task3/img_test/image1.jpg")
        self.poster_image2 = cv2.imread("src/hw3/task3/img_test/image2.jpg")

        self.detector = dlib.get_frontal_face_detector()
        #self.predictor = dlib.shape_predictor('path_to_shape_predictor_model.dat')
        self.face_recognizer = dlib.face_recognition_model_v1('src/hw3/task3/img_test/dlib_face_recognition_resnet_model_v1.dat')

    def extract_face_embeddings(self, image):
        # Detect faces in the image
        
       # color_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        face_embedding = self.face_recognizer.compute_face_descriptor(image)

        
        return face_embedding

    def compare_images(self, image1, image2):
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
       
