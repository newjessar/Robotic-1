#!/usr/bin/env python

## Originally written by Anton Mulder
## Modified by Rik Timmers and Marc Groefsema
import rospy
import cv2
import yaml
from Tkinter import Tk, Button, Scale, HORIZONTAL, Frame, Label, Text, END
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import os
from std_srvs.srv import Empty, SetBool
import rospkg 
#from utils import preprocess_image
from lane_follower import LaneFollower

class Calibrator(object):
    
    def __init__(self):


        self.rospack = rospkg.RosPack()
        self.gui = Tk()
        self.gui.title("Calibrator")
        self.cv_bridge = CvBridge()
        
        self.window = Frame(self.gui, width = 400, height = 500).pack()
        
        slider_length = 200;
        
        self.h_lower_scale = Scale(self.gui, length = slider_length, from_ = 0, to_ = 180, orient = HORIZONTAL)
        self.h_lower_scale.place(x = 100, y = 10)
        self.h_lower_label = Label(self.gui, text = "H lower value:").place(x = 10, y = 30)
        
        self.h_upper_scale = Scale(self.gui, length = slider_length, from_ = 0, to_ = 180, orient = HORIZONTAL)
        self.h_upper_scale.place(x = 100, y = 50)
        self.h_upper_scale.set(180)
        self.h_uppper_label = Label(self.gui, text = "H upper value:").place(x = 10, y = 70)
        
        self.s_lower_scale = Scale(self.gui, length = slider_length, from_ = 0, to_ = 255, orient = HORIZONTAL)
        self.s_lower_scale.place(x = 100, y = 130)
        self.s_lower_label = Label(self.gui, text = "S lower value:").place(x = 10, y = 150)
        
        self.s_upper_scale = Scale(self.gui, length = slider_length, from_ = 0 , to_ = 255, orient = HORIZONTAL)
        self.s_upper_scale.place(x = 100, y = 170)
        self.s_upper_scale.set(255)
        self.s_upper_label = Label(self.gui, text = "S upper value:").place(x = 10, y = 190)
        
        self.v_lower_scale = Scale(self.gui, length = slider_length, from_ = 0, to_ = 255, orient = HORIZONTAL)
        self.v_lower_scale.place(x = 100, y = 250)
        self.v_lower_label = Label(self.gui, text = "V lower value:").place(x = 10, y = 270)
        
        self.v_upper_scale = Scale(self.gui, length = slider_length, from_ = 0, to_ = 255, orient = HORIZONTAL)
        self.v_upper_scale.place(x = 100, y = 290)
        self.v_upper_scale.set(255)
        self.v_upper_label = Label(self.gui, text = "V upper value").place(x = 10, y = 310)
               
        self.dilation_steps = Scale(self.gui, length = slider_length, from_ = 0, to = 30, orient = HORIZONTAL)
        self.dilation_steps.place(x = 100, y = 330) 
        self.dilation_steps.set(0)
        self.dilation_steps.label = Label(self.gui, text = "Dilation").place(x = 10, y = 350)

        self.erosion_steps = Scale(self.gui, length = slider_length, from_ = 0, to = 30, orient = HORIZONTAL)
        self.erosion_steps.place(x = 100, y = 370) 
        self.erosion_steps.set(0)
        self.erosion_steps.label = Label(self.gui, text = "Erosion").place(x = 10, y = 390)
     
        self.close_button = Button(self.gui, text = "Quit", command = self.gui.quit)
        self.close_button.place(x = 180, y = 450)
                
        self.update_loop()
        self.gui.mainloop()
        
        
    def update_loop(self):
        image_msg = rospy.wait_for_message("/camera/image_raw", Image)
        image = self.cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        image = self.filter_image(image)
        cv2.imshow("Image", image)
        cv2.waitKey(1)
        self.gui.after(100, self.update_loop)
        

    #def to_hsv(self, image):
    #    # Convert to HSV 
    #    # Return hsv_image
    #    pass

    #def filter_hsv(self, hsv_image, lower_hsv, upper_hsv):
    #    
    #    # Filter out everything but the red signs
    #    # Return filtered_hsv_image
    #    pass

    def filter_image(self, image):
        kernel = np.ones((5,5),np.uint8)
        dilation_steps = self.dilation_steps.get()
        erosion_steps = self.erosion_steps.get()

        hsv_image  = LaneFollower.to_hsv(image)
        lower_hsv = np.asarray([self.h_lower_scale.get(), self.s_lower_scale.get(), self.v_lower_scale.get()])
        upper_hsv = np.asarray([self.h_upper_scale.get(), self.s_upper_scale.get(), self.v_upper_scale.get()])

        mask = LaneFollower.filter_hsv(hsv_image, lower_hsv, upper_hsv)
        mask = cv2.dilate(mask,kernel,iterations = dilation_steps)
        mask = cv2.erode(mask, kernel, iterations = erosion_steps)
        filtered_image = cv2.bitwise_and(hsv_image, hsv_image, mask = mask)
        filtered_rgb_image = cv2.cvtColor(filtered_image, cv2.COLOR_HSV2BGR)
        return filtered_rgb_image

rospy.init_node("calibrator")
calibrator = Calibrator()
