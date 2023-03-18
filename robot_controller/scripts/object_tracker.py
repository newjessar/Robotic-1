import rospy 
import numpy as np

class ObjectTracker(object):

    def __init__(self):
        self.lane_width = 0.5 # The width of a lane (from line to line) in meters
        self.lane_occupied = {}
        self.lane_occupied["left_lane"] = False
        self.lane_occupied["right_lane"] = False
        self.lane_occupied["front"] = False

########################################################
# OLD CODE - Requires revision !!!!!!!!!!!!!!!!!!!!     
########################################################
    def tracking_objects(self, means):
        # x-axis "item[0]" object on front and back
        # y-axis "item[1]" object on the left and right
        # means = [[-0.81026505, -0.0379693], [0.41456199, -0.07238082], [-0.18267367,  0.3580764]]

        results = []
        # print("means:", means)

        for item in means:
            x, y = item

            if -0.1 <= y <= 0.1:
                if 0 <= x <= 2.5:
                    results.append("on_front")
                elif -2.5 <= x <= 0:
                    results.append("on_back")
                else:
                    results.append(5)

            if 0.25 <= y <= 0.75:
                if not (0.30 >= y >= 0.38):
                    results.append("on_left")

            if -0.75 <= y <= -0.25:
                if not (-0.37 <= y <= -0.30):
                    results.append("on_right")

        return np.unique(results)

          
########################################################
# OLD CODE - Requires revision !!!!!!!!!!!!!!!!!!!!     
########################################################