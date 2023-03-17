import rospy 

class ObjectTracker(object):

  def __init__(self):
    self.lane_width = 0.5 # The width of a lane (from line to line) in meters
    self.lane_occupied = {}
    self.lane_occupied["left_lane"] = False
    self.lane_occupied["right_lane"] = False
    self.lane_occupied["front"] = False

    