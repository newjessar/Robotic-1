import rospy
import cv2 as cv2
import numpy as np
import tensorflow as tf
import sys
import os


class SignRecognizer:
    
    def __init__(self, data_collection_mode = False, data_filename = None):

        self.labels = ["left", "right", "up", "square", "triangle", "smiley", "background"]
        # Data should be stored in /home/username/data/
        self.path = os.path.join(os.environ["HOME"], "data")
        self.ROI = []
        self.roi_classification_size = 28
        self.collectedROIs = []

        self.collect = data_collection_mode
        self.data_filename = data_filename
        
        if self.collect: # If collecting data, run save function on shutdown
            rospy.on_shutdown(self.save_data)
        else: # Else load the network and run the predict function once
            self.model = tf.keras.models.load_model(os.path.join(os.environ["HOME"] + "/network_model", "model_classifier.h5"))
            # Run the network once since first time it is slow
            fake_image = np.zeros((self.roi_classification_size, self.roi_classification_size, 3))
            self.model.predict(np.asarray([fake_image]))


    def to_hsv(self, image):
        # Convert to HSV 
        # Return hsv_image
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        return hsv_image


    def filter_hsv(self, hsv_image):
        # Filter out everything but the red signs
        # Return filtered_hsv_image
        red_lower = np.array([0, 200, 85])
        red_upper = np.array([100, 255, 200])

        filtered_image = cv2.inRange(hsv_image, red_lower, red_upper)
        return filtered_image
    

    # Post process the image, using dilate and erode 
    def post_process(self, image):
        kernel = np.ones((5,5),np.uint8)
        dilation_steps = 7
        erosion_steps = 8
    
        mask = cv2.dilate(image,kernel,iterations = dilation_steps) 
        mask = cv2.erode(mask, kernel, iterations = erosion_steps) 
        masked_image = cv2.bitwise_and(image, image, mask = mask)

        return masked_image

    # Finds the contours (i.e. the rectangles indicating a possible sign)
    def find_contours(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
        return contours
      

    # Extract the ROIs from the image
    def get_rois(self, processed_mask, image):
        out_ROIs = []
        contours = self.find_contours(processed_mask)
        
        for contour in contours:
            # Get the bounding rectangle of the contour
            x, y, w, h = cv2.boundingRect(contour)

            # Filter out too small and too large signs
            if 65 < w < 170 and 65 < h < 170:
                # Extract and resize the ROI from the image
                roi = cv2.resize(image[y: y + h, x: x + w], (self.roi_classification_size, self.roi_classification_size), interpolation=cv2.INTER_AREA)

                # Append the ROI to the list of ROIs
                out_ROIs.append(roi)

        # Get the largest ROI index
        if out_ROIs:
            max_index = np.argmax([roi.shape[0] * roi.shape[1] for roi in out_ROIs])

            # Show the ROI
            cv2.imshow("ROI", out_ROIs[max_index])
            cv2.waitKey(1)

            # print("out_ROIs", out_ROIs[max_index])
            return np.asarray(out_ROIs[max_index])
        return np.asarray(out_ROIs)

    
    # Predict the ROI image and return the label
    def predict(self, roi):
    # Run the self.model.predict function with the input of the ROI image, remember that the input should be normalized
    # Return the label of the ROI image 
        flouting_ROI = np.asarray(roi).astype(np.float32)
        normalize_ROI = np.true_divide(flouting_ROI, 255)
        label_images = self.model.predict(normalize_ROI)
    
        return label_images


    def process(self, image):
        pass

    # # Classify and save the data to a file
    def classify(self, image):
    

        label_images = 0
        label_idx = -1

        # If self.collect == False
        #     Return either 1 ROI, or return the label of that ROI 

    
        self.labels = ["left", "right", "up", "square", "triangle", "smiley", "background"]

        if image is None:
            # Handle the case when the image is not valid
            print("Invalid image format")
        else:
            # Convert and filter HSV 
            hsv_image = self.to_hsv(image)
            # Filter the images to keep only the red color
            mask = self.filter_hsv(hsv_image)
            # Post process the image to remove noise
            processed_mask = self.post_process(mask)
            # Get the largest ROI from the list of ROIs
            roi_array = self.get_rois(processed_mask, image)
            # print ("ROI LIST", roi_array)


            if roi_array.any() and not self.collect:
                label_images = self.predict(roi_array)
                label_idx = np.argmax(label_images)
                # print the sign 
                print(self.labels[label_idx])




            # If self.collect == True
            # Append the ROI image to self.collectedROIs (it will be saved once the node closes)

            if roi_array.any() and self.collect:    
                [self.collectedROIs.append(roi) for roi in roi_array]


            # Return the label of the ROI
            return label_idx
            



        # showing the warped image
        # self.show_image("warped_image_copy", mask)

        # Use the mask to find the contours (which should be rectangles indicating a sign)
        # For each contour extract the rectangle (ROI, Region of Interest)
        #   Filter out to small and to large signs, the robot should not react to signs that are far away.
        #   To get the rectangle use [x, y, w, h] = cv2.boundingRect(countour) 
        #   To extract a ROI from an image use image[y1: y2, x1: x2] 
        # Resize the extracted ROI to self.roi_classification_size x self.roi_classification_size
        # If collecting data, 
        # Else classify the image using the trained CNN.
        #   return the found label(s) 
        


    # When data collection mode is one, the data will be saved on a numpy file when the program is closed
    def save_data(self):
        data = np.asarray(self.collectedROIs)
        # Check if path exists, else create it.
        if not os.path.exists(self.path): os.mkdir(self.path)
        # Save the data
        np.save(os.path.join(self.path, "{0}.npy".format(self.data_filename)), data)
        print ("Saved data")

