import rospy
import cv2
import numpy as np
import tensorflow as tf
import sys
import os

class SignRecognizer:
    
    def __init__(self, data_collection_mode = False, data_filename = None):
        self.labels = ["left", "right", "up", "square", "triangle", "smiley", "background"]
        # Data should be stored in /home/username/data/
        self.path = os.path.join(os.environ["HOME"], "data")
        self.ROI = None
        self.roi_classification_size = 40
        self.collectedROIs = []

        self.collect = data_collection_mode
        self.data_filename = data_filename
        
        if data_collection_mode: # If collecting data, run save function on shutdown
            rospy.on_shutdown(self.save_data)
        else: # Else load the network and run the predict function once
            self.model = tf.keras.models.load_model(os.path.join(os.environ["HOME"] + "/network_model", "model_classifier.h5"))
            # Run the network once since first time it is slow
            fake_image = np.zeros((self.roi_classification_size, self.roi_classification_size, 3))
            self.model.predict(np.asarray([fake_image]))
    
    def to_hsv(self, image):
        # Convert to HSV 
        # Return hsv_image
        pass

    def filter_hsv(self, hsv_image):
        # Filter out everything but the red signs
        # Return filtered_hsv_image
        pass

    # Finds the contours (i.e. the rectangles indicating a possible sign)
    def find_contours(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
        return contours
        
    def classify(self, image):
        # Convert and filter HSV 
        mask = self.filter_hsv(image)
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.dilate(mask,kernel,iterations = 0) # set the number of iterations (use the calibrator to find a good parameter)
        mask = cv2.erode(mask, kernel, iterations = 0) # set the number of iterations (use the calibrator to find a good parameter)
        # Use the mask to find the contours (which should be rectangles indicating a sign)
        # For each contour extract the rectangle (ROI, Region of Interest)
        #   Filter out to small and to large signs, the robot should not react to signs that are far away.
        #   To get the rectangle use [x, y, w, h] = cv2.boundingRect(countour) 
        #   To extract a ROI from an image use image[y1: y2, x1: x2] 
        # Resize the extracted ROI to self.roi_classification_size x self.roi_classification_size
        # If collecting data, append the ROI image to self.collectedROIs (it will be saved once the node closes)
        # Else classify the image using the trained CNN.
        #   return the found label(s) 
        pass

    
    # When data collection mode is one, the data will be saved on a numpy file when the program is closed
    def save_data(self):
        data = np.asarray(self.collectedROIs)
        np.save(os.path.join(self.path, "{0}.npy".format(self.data_filename)), data)
        print "Saved data"


