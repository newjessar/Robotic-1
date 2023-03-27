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

        if self.array_of_signs:
            rospy.on_shutdown(self.to_NPZ(self.array_of_signs))

        
        if self.collect: # If collecting data, run save function on shutdown
            rospy.on_shutdown(self.save_data)
        else: # Else load the network and run the predict function once
            self.model = tf.keras.models.load_model(os.path.join(os.environ["HOME"] + "/network_model", "model_classifier.h5"))
            # Run the network once since first time it is slow
            fake_image = np.zeros((self.roi_classification_size, self.roi_classification_size, 3))
            self.model.predict(np.asarray([fake_image]))
    
    def to_NPZ(self, data):
        data_array = np.asarray(self.array_of_signs)
        # np.savez('arrays.npz', data_array)


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
        
    # Show the image
    def show_image(self, name, image):
        cv2.imshow(name, image)
        key = cv2.waitKey(1) & 0xFF



    def get_rois(self, filtered_image, image):
        contours = self.find_contours(filtered_image)
        arr_rois = []
        # max_x = -1
        # min_x = 1000000
        # print ("image dimensions: ", image.shape[1])
        # print(">>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<")
        # for contour in contours:
        #     [x, y, w, h] = cv2.boundingRect(contour)
        #     if 65 < w < 170 and 65 < h < 170:
        #         # print("x: ", x, "y: ", y, "w: ", w, "h: ", h)

        #         if x > max_x:
        #             max_x = x
        #         else:
        #             min_x = x
        #         # print("max_x: ", max_x, "min_x: ", min_x)
        # print(">>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<")


        # cv2.line(image, (max_x, 0), (max_x, 800), (0, 0, 255), 1)
        # cv2.line(image, (min_x, 0), (min_x, 800), (255, 255, 255), 1)
        # self.show_image("limited_image", image)

        # if 65 < w < 170 and 65 < h < 170:
            

  

        # # Set the trimming of the image in minimum and maximum horizontal limits for traffic signs ralative to
        # # distance of the counters
       
        # min_x_limit = int(image.shape[1] * 0.3)  # 30% from the left side of the image
        # max_x_limit = int(image.shape[1] * 0.9)  # 90% from the left side of the image
        # # print("Image shape: ", image.shape, "min_x_limit: ", min_x_limit, "max_x_limit: ", max_x_limit)

        # # Create and show the limited image
        # limited_image = image[:, min_x_limit:max_x_limit]
        
        width_image_half = image.shape[1] // 2  # X = 1600 / 2 = 800
        height_image_half = image.shape[0] * 0.5  # Y = 800 / 2 = 400
        
        # Loop through the contours and extract the ROIs
        for contour in contours:
            [x, y, w, h] = cv2.boundingRect(contour)
            if 65 < w < 170 and 65 < h < 170:
                print("x: ", x, "y: ", y, "w: ", w, "h: ", h)
                if x:
                    self.array_of_signs.append([x, y, w, h])
                


   

            aspect_ratio = w / h
            aspect_ratio_tolerance = 0.1

            # Common constraints for all cases
            if 65 < w < 170 and 65 < h < 170 and y < height_image_half:

                # Left lane
                if x < width_image_half:
                    if (
                        (0.65 - aspect_ratio_tolerance) < aspect_ratio < (0.65 + aspect_ratio_tolerance) or
                        (1.74 - aspect_ratio_tolerance) < aspect_ratio < (1.74 + aspect_ratio_tolerance)
                    ):
                        print("Left side traffic sign")
                    else:
                        continue

                # Right lane
                else:
                    if (
                        (1.00 - aspect_ratio_tolerance) < aspect_ratio < (1.00 + aspect_ratio_tolerance) or
                        (2.46 - aspect_ratio_tolerance) < aspect_ratio < (2.46 + aspect_ratio_tolerance)
                    ):
                        print("Right side traffic sign")
                    else:
                        continue

            

                 

                # Check if the traffic sign is within the horizontal limits
                # if min_x_limit <= x <= max_x_limit:
                    cropped_image = image[y:y+h, x:x+w]
                    size = (self.roi_classification_size, self.roi_classification_size)
                    output = cv2.resize(cropped_image, size, interpolation=cv2.INTER_AREA)
                    arr_rois.append(output)

                    cv2.imshow("roi", output)
                    cv2.waitKey(1)

        data_array = np.asarray(self.array_of_signs)
        np.savez('arraysSINGLE_LS.npz', data_array)    

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
            else:
                label_idx = None


            # If self.collect == True
            # Append the ROI image to self.collectedROIs (it will be saved once the node closes)
            # if roi_array and self.collect:    
                # [self.collectedROIs.append(roi) for roi in roi_array]
            for item in roi_array:
                self.collectedROIs.append(item)

            # Return the label of the ROI
            return label_idx if label_idx != None else None


    # When data collection mode is one, the data will be saved on a numpy file when the program is closed
    def save_data(self):
        data = np.asarray(self.collectedROIs)
        # Check if path exists, else create it.
        if not os.path.exists(self.path): os.mkdir(self.path)
        # Save the data
        np.save(os.path.join(self.path, "{0}.npy".format(self.data_filename)), data)
        print ("Saved data")

