import rospy 
import numpy as np

class ObjectTracker(object):

    def __init__(self):
        self.lane_width = 0.5 # The width of a lane (from line to line) in meters
        self.half_lane = self.lane_width/2
        self.quarter_segment = self.half_lane/2
        self.lane_occupied = {}
        self.lane_occupied["left_lane"] = False
        self.lane_occupied["right_lane"] = False
        self.lane_occupied["front"] = False
        self.lane_occupied["back_left"] = False
        self.lane_occupied["back_right"] = False


    # tracking objects, take the cluster means as input and return the lane occupied
    # the function will remove doplicate objects 
    def check_lanes(self, clusters):
        # Reset lane_occupied values
        self.lane_occupied["left_lane"] = False
        self.lane_occupied["right_lane"] = False
        self.lane_occupied["front"] = False
        self.lane_occupied["back_left"] = False
        self.lane_occupied["back_right"] = False

        ### !!There is some noise leading to declare an object  detection related to the signs!!
        ### !!to avoide this matter I excluded the size of the signs from each side!!
        ### This issue can appear especially when the vehicle is taken a turn
        if clusters:
            for item in clusters:
                x, y = item
                # print("x: ", x,"y: ", y)
                # object on front
                if -(self.quarter_segment + self.half_lane) <= y <= (self.quarter_segment + self.half_lane):
                    # object on front within 2.0 meters
                    if 0 <= x <= self.lane_width*4:
                        self.lane_occupied["front"] = True
         
                # object on the left and right
                if self.half_lane <= y <= (self.half_lane+self.lane_width):
                    # object on the left
                    if not (0.30 >= y >= 0.38):
                        self.lane_occupied["left_lane"] = True
                        print(">>>>>>>>>>>>>>left side")

                # object on the right
                if -(self.half_lane+self.lane_width) <= y <= -(self.half_lane):
                    if not (-0.37 <= y <= -0.30):
                        self.lane_occupied["right_lane"] = True
                        print(">>>>>>>>>>>>>>right side")
            
                if -(self.lane_width*4) <= x <= 0:
                    # object on the back-left
                    if 0 <= y <= (self.half_lane+self.lane_width):
                        print("back left x: ", x,"y: ", y)
                        self.lane_occupied["back_left"] = True

                    # object on the back-right
                    if -(self.half_lane+self.lane_width) <= y <= 0:
                        print("Back right x: ", x,"y: ", y)
                        self.lane_occupied["back_right"] = True


                    

        

# robot on right lane || bot on right lane: 0.25 >= y >= -0.25
# robot on left lane || bot on right lane: -0.25 >= y >= -0.75
# 0.25 >= y <= 0.75
                               
# robot on right lane || bot on left lane: 0.25 <= y <= -0.25  
# robot on left lane || bot on left lane: -0.25 <= y <= 0.25
# -0.25 >= y >= -0.75




          
