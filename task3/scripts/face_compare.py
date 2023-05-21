#!/usr/bin/python
#coding=utf-8
# The contents of this file are in the public domain. See LICENSE_FOR_EXAMPLE_PROGRAMS.txt
#
#   This example shows how to use dlib's face recognition tool.  This tool maps
#   an image of a human face to a 128 dimensional vector space where images of
#   the same person are near to each other and images from different people are
#   far apart.  Therefore, you can perform face recognition by mapping faces to
#   the 128D space and then checking if their Euclidean distance is small
#   enough.
#
#   When using a distance threshold of 0.6, the dlib model obtains an accuracy
#   of 99.38% on the standard LFW face recognition benchmark, which is
#   comparable to other state-of-the-art methods for face recognition as of
#   February 2017. This accuracy means that, when presented with a pair of face
#   images, the tool will correctly identify if the pair belongs to the same
#   person or is from different people 99.38% of the time.
#
#   Finally, for an in-depth discussion of how dlib's tool works you should
#   refer to the C++ example program dnn_face_recognition_ex.cpp and the
#   attendant documentation referenced therein.
#
#
#
#
# COMPILING/INSTALLING THE DLIB PYTHON INTERFACE
#   You can install dlib using the command:
#       pip install dlib
#
#   Alternatively, if you want to compile dlib yourself then go into the dlib
#   root folder and run:
#       python setup.py install
#   or
#       python setup.py install --yes USE_AVX_INSTRUCTIONS
#   if you have a CPU that supports AVX instructions, since this makes some
#   things run faster.  This code will also use CUDA if you have CUDA and cuDNN
#   installed.
#
#   Compiling dlib should work on any operating system so long as you have
#   CMake installed.  On Ubuntu, this can be done easily by running the
#   command:
#       sudo apt-get install cmake
#
#   Also note that this example requires scikit-image which can be installed
#   via the command:
#       pip install scikit-image
#   Or downloaded from http://scikit-image.org/download.html.
import cv2
import sys
import os
import dlib
import glob
from skimage import io
import numpy as np
import time
from math import sqrt
import os
"""
if len(sys.argv) != 6:
    print(
        "Call this program like this:\n"
        "   ./comparation.py shape_predictor_5_face_landmarks.dat dlib_face_recognition_resnet_model_v1.dat path2face1 path2face2 path2result\n"
        "You can download a trained facial shape predictor and recognition model from:\n"
        "    http://dlib.net/files/shape_predictor_5_face_landmarks.dat.bz2\n"
        "    http://dlib.net/files/dlib_face_recognition_resnet_model_v1.dat.bz2")
    exit()
"""

predictor_path = '/home/kuznerjaka/catkin_ws/src/hw3/task3/img_test/shape_predictor_5_face_landmarks.dat'
face_rec_model_path = '/home/kuznerjaka/catkin_ws/src/hw3/task3/img_test/dlib_face_recognition_resnet_model_v1.dat'
image_path1 = '../img_test/image_cylinder.jpg'
image_path2 = '../img_test/image2.jpg'

class ImageCompareManager():
    print(os.getcwd())
    def __init__(self):
        self.detector = dlib.get_frontal_face_detector()
        self.sp = dlib.shape_predictor(predictor_path)
        self.facerec = dlib.face_recognition_model_v1(face_rec_model_path)
        #self.face_reidentifier = dlib.face_recognition_model_v1('src/hw3/task3/img_test/dlib_face_recognition_resnet_model_v1.dat')
        #face_recognizer = cv2.face.LBPHFaceRecognizer_create()
        #face_recognizer.read('path/to/face_recognition_model.xml')

    def calc_confidence(self, img1, img2) -> float:
        """
        Calculate the confidence (similarity) between two images
        params:
            img1, img2: <numpy.ndarray>
        """
        #print("TYPE:", type(img1))
        shape1 = self.sp(img1, dlib.rectangle(0, 0, img1.shape[0], img1.shape[1]))
        face_descriptor1 = self.facerec.compute_face_descriptor(img1, shape1)

        shape2 = self.sp(img2, dlib.rectangle(0, 0, img2.shape[0], img2.shape[1]))
        face_descriptor2 = self.facerec.compute_face_descriptor(img2, shape2)

        confidence = 1 - np.linalg.norm(np.array(face_descriptor1) - np.array(face_descriptor2))
        height = min(img1.shape[0], img2.shape[0])  # Choose the minimum height
        img1_resized = cv2.resize(img1, (img1.shape[1], height))
        img2_resized = cv2.resize(img2, (img2.shape[1], height))

        # Concatenate the resized images horizontally
        hmerge = np.hstack((img1_resized, img2_resized))
        cv2.putText(hmerge, "confident = " + str(1-val), (10,10), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255, 255, 255), 1)
        cv2.imshow("result", hmerge)
        #cv2.imshow("test2", vmerge)
        cv2.imwrite(sys.argv[5] ,hmerge)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


        return confidence
    
    def compare_faces(self, image1, image2) -> bool:
        confidence = self.calc_confidence(image1, image2)
        print("Confidence that the images are the same:", confidence)
        if confidence > 0.4:
            return True
        else:
            return False

def euclidean_dist(vector_x, vector_y):
    if len(vector_x) != len(vector_y):
        raise Exception('Vectors must be same dimensions')
    return sum((vector_x[dim] - vector_y[dim]) ** 2 for dim in range(len(vector_x)))

def main():
    icm = ImageCompareManager()
    
    confidence = icm.calc_confidence(image_path1, image_path2)
    print("Confidence:", confidence)
 
if __name__ == '__main__':
    main()

