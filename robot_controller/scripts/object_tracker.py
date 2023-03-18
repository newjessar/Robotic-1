import rospy 
import numpy as np

class ObjectTracker(object):

    def __init__(self):
        self.lane_width = 0.5 # The width of a lane (from line to line) in meters
        self.lane_occupied = {}
        self.lane_occupied["left_lane"] = False
        self.lane_occupied["right_lane"] = False
        self.lane_occupied["front"] = False

    # tracking objects, take the cluster means as input and return the lane occupied
    # the function will remove doplicate objects 
    def tracking_objects(self, means):
        # Reset lane_occupied values
        self.lane_occupied["left_lane"] = False
        self.lane_occupied["right_lane"] = False
        self.lane_occupied["front"] = False
        self.lane_occupied["back"] = False

        if means:
            for item in means:
                x, y = item
                # object on the front or back
                if -0.1 <= y <= 0.1:
                    # object on front within 2.5 meters
                    if 0 <= x <= 2.5:
                        self.lane_occupied["front"] = True
                    # object on back within 2.5 meters
                    if -2.5 <= x <= 0:
                        self.lane_occupied["back"] = True

                # object on the left and right
                if 0.25 <= y <= 0.75:
                    # object on the left
                    if not (0.30 >= y >= 0.38):
                        self.lane_occupied["left_lane"] = True
                # object on the right
                if -0.75 <= y <= -0.25:
                    if not (-0.37 <= y <= -0.30):
                        self.lane_occupied["right_lane"] = True


          
