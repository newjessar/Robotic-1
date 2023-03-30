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
        self.ROI = None
        self.roi_classification_size = 28
        self.collectedROIs = []
        self.array_of_signs = []

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
        red_lower = np.array([0, 120, 70])
        red_upper = np.array([10, 255, 255])

        filtered_image = cv2.inRange(hsv_image, red_lower, red_upper)
        return filtered_image
    

    # Post process the image, using dilate and erode 
    def post_process(self, image):
        kernel = np.ones((5,5),np.uint8)
        dilation_steps = 4
        erosion_steps = 4
    
        mask = cv2.dilate(image,kernel,iterations = dilation_steps) 
        mask = cv2.erode(mask, kernel, iterations = erosion_steps) 
        masked_image = cv2.bitwise_and(image, image, mask = mask)

        return masked_image

    # Finds the contours (i.e. the rectangles indicating a possible sign)
    def find_contours(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
        return contours
        
    # Show the image
    def show_image(self, name, image):
        cv2.imshow(name, image)
        key = cv2.waitKey(1) & 0xFF



    def get_rois(self, filtered_image, image):

        contours = self.find_contours(filtered_image)
        arr_rois = []
        # self.show_image("image", image)
        # Loop through the contours and extract the ROIs
        for contour in contours:
            [x, y, w, h] = cv2.boundingRect(contour)
            if 96 < w < 155 and 96 < h < 155:
                if x:
                    self.array_of_signs.append([x, y, w, h])
                if 312 < x < 1180:
                    if (x > 1160 and y > 200):
                        continue
                    else:
                        # print("x: ", x, "y: ", y, "w: ", w, "h: ", h)
                        cropped_image = image[y:y+h, x:x+w]
                        size = (self.roi_classification_size, self.roi_classification_size)
                        output = cv2.resize(cropped_image, size, interpolation=cv2.INTER_AREA)
                        arr_rois.append(output)
                        # Shwoing the ROI
                        cv2.imshow("roi", output)
                        cv2.waitKey(1)
        # Save the data for further Analysis
        # data_array = np.asarray(self.array_of_signs)
        # np.savez('bROIRL.npz', data_array)    

        return arr_rois

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

        # If self.collect == False
        #     Return either 1 ROI, or return the label of that ROI 
        # self.labels = ["left", "right", "up", "square", "triangle", "smiley", "background"]
        # self.labels = [" 1 ",  "  2  ", "3 ", "   4  ", "    5   ", "   6  ", "    7     "]

        # print("running process")
        if image is None:
            # Handle the case when the image is not valid
            print("Invalid image format")
        else:
            # Convert and filter HSV 
            label_images = 0
            label_idx = None
            hsv_image = self.to_hsv(image)
            # Filter the images to keep only the red color
            mask = self.filter_hsv(hsv_image)
            # Post process the image to remove noise
            processed_mask = self.post_process(mask)
            # Get the largest ROI from the list of ROIs
            roi_array = self.get_rois(processed_mask, image)
            # print ("ROI LIST", roi_array)


            if roi_array and not self.collect:
                label_images = self.predict(roi_array)
                label_idx = np.argmax(label_images)
                # print the sign 
                # print("label_idx", label_idx)
                # print(self.labels[label_idx])

            # If self.collect == True
            # Append the ROI image to self.collectedROIs (it will be saved once the node closes)
            if roi_array and self.collect:
                for item in roi_array:
                    self.collectedROIs.append(item)

            # Return the label of the ROI
            return label_idx+1 if label_idx != None else None


    # When data collection mode is one, the data will be saved on a numpy file when the program is closed
    def save_data(self):
        data = np.asarray(self.collectedROIs)
        # Check if path exists, else create it.
        if not os.path.exists(self.path): os.mkdir(self.path)
        # Save the data
        np.save(os.path.join(self.path, "{0}.npy".format(self.data_filename)), data)
        print ("Saved data")

