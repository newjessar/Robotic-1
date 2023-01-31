from sensor_msgs.msg import LaserScan
import rospy

class LaserData(object):

  def __init__(self, callback_function):
    # The callback function should be defined in the Controller
    self.laser_scan_subscriber = rospy.Subscriber("/scan", LaserScan, callback_function, queue_size=1) 

  def convert_to_cartesian(self, laser_msg):
    # Convert the laser polar coordinates to cartesian coordinates
    # Return the cartesian data
    pass
  
  def distance(self, point1, point2):
    # Return the Euclidean distance between the two points
    pass
    
  def cluster(self, data):
    # Cluster the data
    # Return the clusters
    pass