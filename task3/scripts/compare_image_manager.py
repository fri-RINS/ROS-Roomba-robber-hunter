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
        self.predictor = dlib.shape_predictor('path_to_shape_predictor_model.dat')
        self.face_recognizer = dlib.face_recognition_model_v1('path_to_face_recognition_model.dat')

    def extract_face_embeddings(self, image):
        # Detect faces in the image
        faces = self.detector(image)
        
        # Iterate over detected faces
        face_embeddings = []
        for face in faces:
            # Align face
            shape = self.predictor(image, face)
            aligned_face = dlib.get_face_chip(image, shape)
            
            # Extract face embedding
            face_embedding = self.face_recognizer.compute_face_descriptor(aligned_face)
            face_embeddings.append(face_embedding)
        
        return face_embeddings

    def compare_images(self, image1, image2):
        # Load the images

        # Convert images to grayscale
        gray1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)

        # # # Resize the faces to a common size for better comparison
        # face1 = cv2.resize(gray1, (160, 160))
        # face2 = cv2.resize(gray2, (160, 160))


        # # Normalize pixel values
        # face1 = face1.astype("float32") / 255.0
        # face2 = face2.astype("float32") / 255.0

        # Extract face embeddings for both images
        embeddings1 = self.extract_face_embeddings(gray1)
        embeddings2 = self.extract_face_embeddings(gray2)

        # Compare feature vectors using cosine similarity
        similarity_scores = []
        for embedding1 in embeddings1:
            for embedding2 in embeddings2:
                similarity_score = cosine_similarity([np.array(embedding1)], [np.array(embedding2)])[0][0]
                similarity_scores.append(similarity_score)

        # Compute average similarity score
        average_similarity_score = np.mean(similarity_scores)
 
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
       
