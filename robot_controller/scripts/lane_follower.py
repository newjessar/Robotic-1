import rospy 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist

class LaneFollower(object):

  def __init__(self, forward_speed):
    self.forward_speed = forward_speed
    self.cv_bridge = CvBridge()
    rospy.on_shutdown(self.on_shutdown)

    cmd_topic = "/cmd_vel"
    image_topic = "/camera/image_raw"

    self.cmd_vel_publisher = rospy.Publisher(cmd_topic, Twist, queue_size=1)
    self.image_subscriber = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1)
    self.original_image = None

  def on_shutdown(self):
    twist_msg = Twist()
    self.cmd_vel_publisher.publish(twist_msg)

  def to_hsv(self, image):
    # Convert to HSV 
    # Return hsv_image
    pass
  
  def filter_hsv(self, hsv_image):
    # Filter out everything but the yellow line
    # Return filtered_hsv_image
    pass
  
  def warp_perspective(self, image):
    # Warp the image, save the warped image as warped_image
    # Crop the warped result in order to only show the relevant parts
    cropped_image = warped_image[500:800, 500:1100] # DO NOT CHANGE 
    return cropped_image
  
  def get_lines(self, hsv_image):
    # use HoughLinesP to get all the lines
    # Filter each line to determine to which lane in belongs to 
    # Return far_left, left, right, far_right
    pass
  
  def get_angles(self, far_left, left, right, far_right):
    # Determine the angle for list of lines
    # Return far_left_angle, left_angle, right_angle, far_right_angle
    pass

  def send_velocity(self): # Add parameters
    # Create a Twist message
    # Limit the rotational velocity to (-)0.5 rad/s, this will help with switching lanes
    # For the forward velocity use the self.forward_velocity variable
    # use the self.cmd_vel_publisher to publish the velocity
    pass

  def show_image(self, name, image):
    cv2.imshow(name, image)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'): # Pressing 'q' will shutdown the program
      rospy.signal_shutdown("exit")

  # Image callback function, gets called at 10Hz 
  def image_callback(self, image_msg):
    # Convert the sensor_msgs Image to OpenCv image
    # An OpenCV image has the color channels in order BGR (instead of the more common RGB)
    image = self.cv_bridge.imgmsg_to_cv2(image_msg, "bgr8") 
    # image is now an OpenCV image

    # store the image, such that it can be used for sign recognition later on.
    self.original_image = image.copy()

    # Warp and crop the image
    # Convert and filter HSV (use the calibrator to find correct HSV filter values)
    # Get lists of all for lines
    # Determine the angles of each line
    # Determine the error 
    # Determine the rotational velocity
    # Send velocity


   