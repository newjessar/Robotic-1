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
        

        # self.show_image("limited_image", image)
        # # self.show_image("image", image)



        contours = self.find_contours(filtered_image)
        arr_rois = []
        # self.show_image("image", image)
        # Loop through the contours and extract the ROIs
        for contour in contours:
            [x, y, w, h] = cv2.boundingRect(contour)
            if 90 < w < 155 and 90 < h < 155:
                print("2- x: ", x, "y: ", y, "w: ", w, "h: ", h)
                if x:
                    self.array_of_signs.append([x, y, w, h])
                if 312 < x < 1100:
                    print(">>>>>>>>>>>>>>>>>>>>>>>>>>> x is here")
                    # if x < 1100 and x > 190: # 1- Excluding the right lane signs while driving on the left lane
                #     # 2- Excluding half of the left lanes signs while driving on the right lane
                #     if (x < 425): # 2- Rest of x values for the left lane
                #         if (280 > y > 130):
                #             print("x: ", x, "y: ", y, "w: ", w, "h: ", h)
                #             print("Excludeded")
                #             continue
                #         else:
                #             print("1- x: ", x, "y: ", y, "w: ", w, "h: ", h)
                #             cropped_image = image[y:y+h, x:x+w]
                #             size = (self.roi_classification_size, self.roi_classification_size)
                #             output = cv2.resize(cropped_image, size, interpolation=cv2.INTER_AREA)
                #             arr_rois.append(output)

                #             cv2.imshow("roi", output)
                #             cv2.waitKey(1)
                #     else:
                    cropped_image = image[y:y+h, x:x+w]
                    size = (self.roi_classification_size, self.roi_classification_size)
                    output = cv2.resize(cropped_image, size, interpolation=cv2.INTER_AREA)
                    arr_rois.append(output)

                    cv2.imshow("roi", output)
                    cv2.waitKey(1)
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
        # min_x_limit = int(image.shape[1] * 0.25)  # 30% from the left side of the image
        # max_x_limit = int(image.shape[1] * 0.75)  # 70% from the left side of the image

        # min_y_limit = 0  # Start from the top of the image
        # max_y_limit = image.shape[0]  # End at the bottom of the image

        # # Create and show the limited image
        # limited_image = image[min_y_limit:max_y_limit, min_x_limit:max_x_limit]

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


# RR
# ('2- x: ', 912, 'y: ', 235, 'w: ', 92, 'h: ', 91)
# ('2- x: ', 937, 'y: ', 209, 'w: ', 107, 'h: ', 106)
# ('sign: ', 5)
# ('2- x: ', 945, 'y: ', 195, 'w: ', 114, 'h: ', 114)
# ('2- x: ', 958, 'y: ', 180, 'w: ', 123, 'h: ', 122)
# ('sign: ', 5)
# ('2- x: ', 977, 'y: ', 154, 'w: ', 137, 'h: ', 137)
# ('2- x: ', 1004, 'y: ', 122, 'w: ', 156, 'h: ', 155)
# ('sign: ', 5)

# RL
# ('2- x: ', 304, 'y: ', 236, 'w: ', 93, 'h: ', 91)
# ('2- x: ', 190, 'y: ', 200, 'w: ', 115, 'h: ', 111)
# ('sign: ', 5)
# ('2- x: ', 115, 'y: ', 175, 'w: ', 131, 'h: ', 125)
# ('2- x: ', 64, 'y: ', 157, 'w: ', 141, 'h: ', 135)
# ('sign: ', 5)
# ('2- x: ', 0, 'y: ', 135, 'w: ', 154, 'h: ', 147)
# ('sign: ', None)


# LL
# ('2- x: ', 566, 'y: ', 236, 'w: ', 91, 'h: ', 91)
# ('2- x: ', 499, 'y: ', 204, 'w: ', 110, 'h: ', 109)
# ('sign: ', 5)
# ('2- x: ', 502, 'y: ', 186, 'w: ', 120, 'h: ', 119)
# ('2- x: ', 500, 'y: ', 175, 'w: ', 126, 'h: ', 125)
# ('sign: ', 5)
# ('2- x: ', 493, 'y: ', 164, 'w: ', 131, 'h: ', 131)
# ('2- x: ', 476, 'y: ', 150, 'w: ', 140, 'h: ', 139)
# ('sign: ', 5)
# ('2- x: ', 239, 'y: ', 35, 'w: ', 153, 'h: ', 91)
# ('sign: ', None)

#LR
# ('2- x: ', 1182, 'y: ', 235, 'w: ', 94, 'h: ', 91)
# ('2- x: ', 1196, 'y: ', 228, 'w: ', 98, 'h: ', 96)
# ('sign: ', 3)
# ('2- x: ', 1227, 'y: ', 213, 'w: ', 107, 'h: ', 104)
# ('2- x: ', 1285, 'y: ', 190, 'w: ', 121, 'h: ', 117)
# ('sign: ', 3)
# ('2- x: ', 1339, 'y: ', 167, 'w: ', 135, 'h: ', 130)
# ('2- x: ', 1382, 'y: ', 147, 'w: ', 146, 'h: ', 141)
# ('sign: ', 3)
