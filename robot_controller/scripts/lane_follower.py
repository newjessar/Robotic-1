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
    
    self.original_image = None
    self.hsv_upper_values = [255,255,255]
    self.hsv_lower_values = [0,0,0]
    pts1 = np.float32([[248, 800], [734, 451], [1418, 800], [884, 451]])
    pts2 = np.float32([[734, 800], [734, 0], [884, 800], [884, 0]])
    self.M_transform = cv2.getPerspectiveTransform(pts1,pts2)

    self.far_left_angle = 0
    self.left_angle = 0
    self.right_angle = 0
    self.far_right_angle = 0
    self.image_subscriber = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1)

  def on_shutdown(self):
    twist_msg = Twist()
    self.cmd_vel_publisher.publish(twist_msg)
  
  @staticmethod
  def to_hsv(image):
    # Convert the BGR image to HSV 
    # Return hsv_image
    pass
    
  @staticmethod
  def filter_hsv(hsv_image, lower_hsv, upper_hsv):
    # Filter out everything but the yellow line
    # Return filtered_hsv_image
    pass
  
  def warp_perspective(self, image):
    height, width, _ = image.shape
    warped_image = cv2.warpPerspective(image, self.M_transform, (width, height)) 
    cropped_image = warped_image[500:800, 500:1100] # DO NOT CHANGE 
    return cropped_image
  
  def get_lines(self, binary_mask):
    # Filter each line to determine to which lane in belongs to 
    # Return far_left, left, right, far_right
    pass
  
  def get_angles(self, far_left, left, right, far_right):
    # Determine the angle for list of lines
    # Store the angles in the class variables self.far_left_angle, self.left_angle, self.right_angle, self.far_right_angle
    pass

  def send_velocity(self, omega): 
    # Create a Twist message
    msg = Twist()
  

    # Limit the rotational velocity to (-)0.5 rad/s, this will help with switching lanes
    # For the forward velocity use the self.forward_velocity variable
    # use the self.cmd_vel_publisher to publish the velocity
    self.cmd_vel_publisher.publish(msg)
    
  ## Draw a blue overlay line on the image
  def draw_line(self, image, line):
    if line is not None:
      x1, y1, x2, y2 = line
      cv2.line(image, (x1, y1), (x2, y2), (255, 0, 0), 2)

  def show_image(self, name, image):
    cv2.imshow(name, image)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'): # Pressing 'q' will shutdown the program
      rospy.signal_shutdown("exit")
    if key == ord('s'): # Pressing 's' will save the image
      cv2.imwrite("lane_follower_image.png", image)


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
    # Determine the error for the P controller. This error should depend on which lane you want to follow.
    # Determine the rotational velocity
    # Send velocity


   