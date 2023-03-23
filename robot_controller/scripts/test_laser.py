import rospy
from sensor_msgs.msg import LaserScan
from laser_data import LaserData
import numpy as np
import matplotlib.pyplot as plt


def distance(point1, point2):
    return np.sqrt(np.sum((np.array(point1) - np.array(point2))**2))


def callback_function(laser_msg):
    global laser_data
    cartesian_data = laser_data.convert_to_cartesian(laser_msg)
    means = laser_data.cluster(cartesian_data)
    clusters = [means]  # Wrap the means in a list
    # print("Clusters:", clusters)  # Add this line
    clusterss = [cartesian_data]
    laser_data.plot_clusters(clusters)  
    # print(clusters)

    ## This code is related to calculating the precentile
    # Adjusted to handle the data structure
    # data = np.array(clusterss[0]) 

    # # Calculate distances between consecutive points
    # distances = [distance(data[i], data[i + 1]) for i in range(len(data) - 1)]

    # # Calculate the threshold using the 90th percentile
    # threshold = np.percentile(distances, 98.1)
    # print("Threshold:", threshold)


    



if __name__ == "__main__":
  rospy.init_node("plot_clusters")
  laser_data = LaserData(callback_function)
  
  rospy.spin()



