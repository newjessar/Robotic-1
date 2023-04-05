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
        self.lane_occupied["back"] = False
        self.lane_occupied["front_buffer"] = False
        self.front_distance = None

    # Find the closest point to the robot
    def distance(self, point1, point2):
        point1 = np.array(point1)
        point2 = np.array(point2)
        dist = np.linalg.norm(point1 - point2)
        return dist

    # tracking objects, take the cluster means as input and return the lane occupied
    # the function will remove doplicate objects 
    def check_lanes(self, clusters):
        # Reset lane_occupied values
        self.lane_occupied["left_lane"] = False
        self.lane_occupied["right_lane"] = False
        self.lane_occupied["front"] = False
        self.lane_occupied["back"] = False
        self.lane_occupied["front_buffer"] = False
        self.front_distance = None

        if clusters:
            for item in clusters:
                x, y = item

                # # object on front within 2.0 meters
                if (0.0 <= x <= 2.0) and (-0.25 < y < 0.25):
                    self.lane_occupied["front"] = True
                    # self.front_distance = self.distance([0, 0], [x, y])
                    if (0.0 <= x <= 1.0):
                        self.lane_occupied["front_buffer"] = True
             

                # # object on the right
                # if (-2.5 <= x <= 2.5) and (-0.25 > y > -0.75):
                #     self.lane_occupied["right_lane"] = True

                # # object on the left
                # if (-2.5 <= x <= 2.5) and (0.25 < y < 0.75):
                #     self.lane_occupied["left_lane"] = True


                # # object on the back-right
                # if (-2.5 <= x <= 0.5) and (-0.25 < y < 0.25):
                #     self.lane_occupied["back"] = True
                #     print("back object")



 

            

          
