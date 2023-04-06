import rospy 
import numpy as np

class ObjectTracker(object):

    def __init__(self):
        self.lane_width = 0.5 # The width of a lane (from line to line) in meters
        self.half_lane = 2.5
        self.quarter_segment = 1.25
        self.lane_occupied = {}
        self.lane_occupied["left_lane"] = False
        self.lane_occupied["right_lane"] = False
        self.lane_occupied["front"] = False
        self.lane_occupied["front_turn"] = False


    # tracking objects, take the cluster means as input and return the lane occupied
    # the function will remove doplicate objects 
    def check_lanes(self, clusters):
        # Reset lane_occupied values
        self.lane_occupied["left_lane"] = False
        self.lane_occupied["right_lane"] = False
        self.lane_occupied["front"] = False
        self.lane_occupied["front_turn"] = False

    
        if clusters:
            for item in clusters:
                x, y = item
                # print("x: ", x, "y: ", y)
                # # object on front within 2.0 meters
                if (0.0 <= x <= 2.0) and (-0.25 < y < 0.25):
                    # print("object in front")
                    self.lane_occupied["front"] = True

                if (0.0 <= x <= 2.0):
                    if ((-0.375 < y < -0.25) or (0.375 > y > 0.25)):
                        # print("object turn in front")
                        self.lane_occupied["front_turn"] = True
             
                # object on the right
                if (-2.5 <= x <= 2.5) and (-0.25 > y > -0.75):
                    self.lane_occupied["right_lane"] = True

                # object on the left
                if (-2.5 <= x <= 2.5) and (0.25 < y < 0.75):
                    self.lane_occupied["left_lane"] = True


    



 

            

          
