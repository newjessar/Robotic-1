import rospy
from sensor_msgs.msg import LaserScan
from laser_data import LaserData


def callback_function(laser_msg):
    global laser_data
    cartesian_data = laser_data.convert_to_cartesian(laser_msg)
    means = laser_data.cluster(cartesian_data)
    clusters = [means]  # Wrap the means in a list
    # print("Clusters:", clusters)  # Add this line
    clusterss = [cartesian_data]
    laser_data.plot_clusters(clusters)  




if __name__ == "__main__":
  rospy.init_node("plot_clusters")
  laser_data = LaserData(callback_function)
  
  rospy.spin()



# x-axis "item[0]" object on front and back
# y-axis "item[1]" object on the left and right

  # [-0.78559368  0.0065482 ]
  # on_back

  # [-0.71753356 -0.05676296]
  # [-0.17664223  2.78580395]
  # [-0.7163555   0.05980693]
  # on_back
