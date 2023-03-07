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
    self.hsv_lower_values = np.array([25, 50, 70])
    self.hsv_upper_values = np.array([35, 255, 255])
    # self.hsv_lower_values = np.array([30,112,80])
    # self.hsv_upper_values = np.array([33,162,255])
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
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    return hsv_image
    
  @staticmethod
  def filter_hsv(hsv_image, lower_hsv, upper_hsv):
    # Filter out everything but the yellow line
    # Return filtered_hsv_image
    filtered_hsv_image = cv2.inRange(hsv_image, lower_hsv, upper_hsv)
    return filtered_hsv_image
  
  
  def warp_perspective(self, image):
    height, width, _ = image.shape
    warped_image = cv2.warpPerspective(image, self.M_transform, (width, height)) 
    cropped_image = warped_image[500:800, 500:1100] # DO NOT CHANGE 
    return cropped_image
  
  # Get all the lines form the HSV image, and separate the lines into far_left_line, 
  # left_line, right_line, far_right_line
  def get_lines(self, binary_mask):
    # Filter each line to determine to which lane in belongs to 
    # Return far_left, left, right, far_right
    # Get lists of all four lines
    _, binary_mask_width, _ = binary_mask.shape
    binary_mask_middle = binary_mask_width / 2
    segmented_image_width = binary_mask_middle / 2
    
    # Declare the lines
    far_left, left, right, far_right = [], [], [], []

    # Find all the lines using cv2.HoughLinesP function
    lines = cv2.HoughLinesP(binary_mask, 1, np.pi/180, 100, minLineLength=100, maxLineGap=10)
    
    # Filter the lines into the four lists
    if lines is not None:
      for line in lines:
        x1, y1, x2, y2 = line[0]
        print(line[0])
        # if x1 < binary_mask_middle - segmented_image_width and x2 < binary_mask_middle - segmented_image_width:
        #       far_left.append(line[0])
        # elif x1 < binary_mask_middle and x2 < binary_mask_middle:
        #   left.append(line[0])
        # elif x1 > binary_mask_middle and x2 > binary_mask_middle:
        #   right.append(line[0])
        # elif x1 > binary_mask_middle + segmented_image_width and x2 > binary_mask_middle + segmented_image_width:
        #   far_right.append(line[0])

    return far_left, left, right, far_right
  
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

  
  # Put text on the image  
  def put_text(self, image, position, text ="", color=(255, 255, 255)):
    cv2.putText(image, text, position, cv2.FONT_HERSHEY_COMPLEX, 0.6, color)
  
  ## Draw a blue overlay line on the image
  def draw_line(self, image, line):
    if line is not None:
      x1, y1, x2, y2 = line
      cv2.line(image, (x1, y1), (x2, y2), (255, 0, 0), 2)
    
  # Draw the four lines (Far left, Left, Right, and Far right), and 
  # put the text of each line on the image
  def draw_info(self, image, middle, width):
    self.draw_line(image, self.far_left)
    self.draw_line(image, self.left)
    self.draw_line(image, self.right)
    self.draw_line(image, self.far_right)
    
    self.put_text(image, (middle - width, 50), "Far Left")
    self.put_text(image, (middle - width, 70), "Left")
    self.put_text(image, (middle - width, 90), "Right")
    self.put_text(image, (middle - width, 110), "Far Right")



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
    warped_image = self.warp_perspective(image)
    _, warped_image_width, _ = warped_image.shape # Get the height, width, channels from the warped image 
    middle = warped_image_width / 2 # Determine the center of the image 
    segment_width = middle / 2 # Determine the width of each segment 

    
    # Convert and filter HSV (use the calibrator to find correct HSV filter values)
    hsv_image = self.to_hsv(warped_image)
    filtered_hsv_image = self.filter_hsv(hsv_image, self.hsv_lower_values, self.hsv_upper_values)




    # Filter the lines to determine which lane they belong to
    lines = self.get_lines(filtered_hsv_image)

    # showing the warped image
    self.show_image("filtered_hsv_image", filtered_hsv_image)

    # Draw the Probabilistic Line Transform. 
    # self.draw_line(filtered_hsv_image, lines)
    # self.draw_info(filtered_hsv_image, warped_image_middle, segmented_image_width)

    
    # Determine the angles of each line
    # Determine the error for the P controller. This error should depend on which 
      #  lane you want to follow.
    # Determine the rotational velocity
    # Send velocity


   