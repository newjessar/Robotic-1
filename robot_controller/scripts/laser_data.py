from sensor_msgs.msg import LaserScan
import rospy
import numpy as np
import math
from math import cos, sin, sqrt


class LaserData(object):

  def __init__(self, callback_function):
        # The callback function should be defined in the Controller
    self.laser_scan_subscriber = rospy.Subscriber("/scan", LaserScan, callback_function, queue_size=1) 


  # Convert the laser polar coordinates to cartesian coordinates
  def convert_to_cartesian(self, laser_msg):
      angle_min = laser_msg.angle_min
      angle_increment = laser_msg.angle_increment

      corData = [(r * cos((i * angle_increment) + angle_min), r * sin((i * angle_increment) + angle_min))
                for i, r in enumerate(laser_msg.ranges) if r != np.inf]

      return corData


  # Find the closest point to the robot
  def distance(self, point1, point2):
      point1 = np.array(point1)
      point2 = np.array(point2)
      dist = np.linalg.norm(point1 - point2)
      return dist

  # Find the closest point to the robot
  def cluster(self, data):
      data = np.array(data)
      threshold = 0.3
      min_points = 10
      
      clusters = []
      current_cluster = [data[0]]
      
      for point in data[1:]:
          if self.distance(current_cluster[-1], point) < threshold:
              current_cluster.append(point)
          else:
              if len(current_cluster) > min_points:
                  clusters.append(current_cluster)
              current_cluster = [point]
      
      if len(current_cluster) > min_points:
          clusters.append(current_cluster)
      
      means = [np.mean(cluster, axis=0) for cluster in clusters]
      
      return means
