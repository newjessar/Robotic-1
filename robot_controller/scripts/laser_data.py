from sensor_msgs.msg import LaserScan
import rospy
import numpy as np
import math
from math import cos, sin, sqrt


class LaserData(object):

  def __init__(self, callback_function):
    # The callback function should be defined in the Controller
    self.laser_scan_subscriber = rospy.Subscriber("/scan", LaserScan, callback_function, queue_size=1)
    self.color_order = [(188,112,0), (23,82,217), (142,47,125), (48,172,119), (47,20,162)] 


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
    # Cluster the data
    # Return the clusters


  def plot_clusters(self, clusters):
    # create the canvas
    canvas = 200 * np.ones((600, 600, 3), dtype=np.uint8)
    x_center = canvas.shape[0] / 2
    y_center = canvas.shape[1] / 2

    for clusterIdx in range(len(clusters)):
      color = self.color_order[clusterIdx % 5]
      for point in clusters[clusterIdx]:
        cv2.circle(canvas, (int(100*point[0] + x_center), int(100*point[1] + y_center)), 1, color, -1)
    
    cv2.arrowedLine(canvas,(0,300),(599,300),(0, 0, 0), tipLength = 0.02) # x-axis
    cv2.arrowedLine(canvas,(300,0),(300,599),(0, 0, 0), tipLength = 0.02) # y-axis

    # flip y-direction and rotate canvas for easy viewing
    canvas = cv2.rotate(cv2.flip(canvas,0), cv2.ROTATE_90_COUNTERCLOCKWISE)

    # add axis labels
    cv2.putText(canvas,'x',(305,590),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,0,0),1,cv2.LINE_AA)
    cv2.putText(canvas,'y',(580,320),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,0,0),1,cv2.LINE_AA)
    cv2.putText(canvas,'-3',(272,588),cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,0,0),1,cv2.LINE_AA)
    cv2.putText(canvas,'-3',(575,290),cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,0,0),1,cv2.LINE_AA)
    cv2.putText(canvas,'3',(282,20),cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,0,0),1,cv2.LINE_AA)
    cv2.putText(canvas,'3',(10,290),cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,0,0),1,cv2.LINE_AA)
    
    cv2.imshow("Lidar clusters", canvas)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'): # Pressing 'q' will shutdown the program
      rospy.signal_shutdown("exit")
    if key == ord('s'): # Pressing 's' will save the image
      cv2.imwrite("lidar_image.png", canvas)
