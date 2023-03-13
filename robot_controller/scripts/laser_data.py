from sensor_msgs.msg import LaserScan
import rospy
import numpy as np
import cv2

class LaserData(object):

  def __init__(self, callback_function):
    # The callback function should be defined in the Controller
    self.laser_scan_subscriber = rospy.Subscriber("/scan", LaserScan, callback_function, queue_size=1)
    self.color_order = [(188,112,0), (23,82,217), (142,47,125), (48,172,119), (47,20,162)] 

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