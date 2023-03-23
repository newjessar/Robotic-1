import rospy 
import numpy as np

class ObjectTracker(object):

    def __init__(self):
        self.lane_width = 0.5 # The width of a lane (from line to line) in meters
        self.lane_segment = self.lane_width/2
        self.lane_occupied = {}
        self.lane_occupied["left_lane"] = False
        self.lane_occupied["right_lane"] = False
        self.lane_occupied["front"] = False

    # tracking objects, take the cluster means as input and return the lane occupied
    # the function will remove doplicate objects 
    def check_lanes(self, clusters):
        # Reset lane_occupied values
        self.lane_occupied["left_lane"] = False
        self.lane_occupied["right_lane"] = False
        self.lane_occupied["front"] = False

        ### !!There is some noise leading to declare an object  detection related to the signs!!
        ### !!to avoide this matter I excluded the size of the signs from each side!!
        if clusters:
            for item in clusters:
                x, y = item
                # object on the front or back
                if -0.1 <= y <= 0.1:
                    # object on front within 2.0 meters
                    if 0 <= x <= 2.0:
                        self.lane_occupied["front"] = True
         
                # object on the left and right
                if self.lane_segment <= y <= (self.lane_segment+self.lane_width):
                    # object on the left
                    if not (0.30 >= y >= 0.38):
                        self.lane_occupied["left_lane"] = True

                # object on the right
                if -(self.lane_segment+self.lane_width) <= y <= -(self.lane_segment):
                    if not (-0.37 <= y <= -0.30):
                        self.lane_occupied["right_lane"] = True


          
