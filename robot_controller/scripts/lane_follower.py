import rospy 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from math import atan2


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

    pts1 = np.float32([[248, 800], [734, 451], [1418, 800], [884, 451]])
    pts2 = np.float32([[734, 800], [734, 0], [884, 800], [884, 0]])
    self.M_transform = cv2.getPerspectiveTransform(pts1,pts2)

    self.far_left_angle = 0
    self.left_angle = 0
    self.right_angle = 0
    self.far_right_angle = 0

    ## Lane swtiching Untility
    self.straight_line_count = 0
    self.switch_left = False
    self.switch_right = False
    self.old_omega = 0.0
    self.left_object = False
    self.right_object = False
    self.left_lane_exist = False
    self.right_lane_exist = False
    self.straight_path = False
    self.left_sign = False
    self.right_sign = False
    self.left_turn_executed = False
    self.right_turn_executed = False

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
  
  # Get all the lines form the HSV image, and separate the lines into far_left, 
  # left, right, far_right
  def get_lines(self, binary_mask):
        
    _, binary_mask_width = binary_mask.shape
    middle_lane = binary_mask_width / 2
    segmented_width = middle_lane / 2

    # # Declare the lines
    far_left, left, right, far_right = [], [], [], []
    # Find all the lines using cv2.HoughLinesP function
    # lines = np.asarray(cv2.HoughLinesP(binary_mask, 0.3, np.pi/180, 70, 100, 5))

    # Assuming 'image' is your grayscale input image
    lsd = cv2.createLineSegmentDetector(0)
    detected_lines, width, prec, nfa = lsd.detect(binary_mask)

    # Round the coordinates to the nearest integer and convert them to int 
    lines = np.round(detected_lines).astype(np.int32)

    ## Remove the extra dimension from the lines array
    lines = np.squeeze(lines)

    ## Itteration over the lines to get the far_left, left, right, far_right lines
    if lines.any():
        # Calculate the average x values for each line
        x_axis = np.mean(lines[:, [0, 2]], axis=1)

        # Create masks for each lane
        far_left_mask = (x_axis > 0) & (x_axis < middle_lane - segmented_width)
        left_mask = (x_axis > middle_lane - segmented_width) & (x_axis < middle_lane)
        right_mask = (x_axis > middle_lane) & (x_axis < middle_lane + segmented_width)
        far_right_mask = x_axis > middle_lane + segmented_width

        # Filter the lines using the masks
        far_left = lines[far_left_mask]
        left = lines[left_mask]
        right = lines[right_mask]
        far_right = lines[far_right_mask]

    return np.asarray(far_left), np.asarray(left), np.asarray(right), np.asarray(far_right)
  
    
  ## Ge the highest and lowest x and y values for a given line
  def get_straight_Line(self, line):
    if line.size > 0:
      # Calculate the sum of x1 and x2 for each line
      x_sums = line[:, 0] + line[:, 2]
      max_line = line[np.argmax(x_sums)]

      return max_line

  ## Get the angle for a given line
  def get_perLine_angle(self, line):
      threshold = 1
      if line.size > 0:
            if abs(abs(line[0]) - abs(line[2])) == threshold:
                mean_x = abs((abs(line[0]) + abs(line[2])) / 2)
                angle = atan2(mean_x - mean_x, abs(line[1]) - abs(line[3]))
            else:
                angle = atan2(abs(line[0]) - abs(line[2]), abs(line[1]) - abs(line[3]))
                
            if abs(angle) < 0.09:
                return 0.0
            else:
              return angle

  # Get the angle for each line
  def get_angles(self, far_left, left, right, far_right):
        
    is_turn = True

    if left.size > 0:
          l_angle = self.get_perLine_angle(self.get_straight_Line(left))
          self.left_angle = l_angle
          is_turn = is_turn and (self.left_angle == 0.0)
    else:
          self.left_angle = 0.0
    
    if right.size > 0:
          r_angle = self.get_perLine_angle(self.get_straight_Line(right))
          self.right_angle = r_angle
          is_turn = is_turn and (self.right_angle == 0.0)            
    else:
          self.right_angle = 0.0
    
    if far_left.size > 0:
          fl_angle = self.get_perLine_angle(self.get_straight_Line(far_left))
          self.far_left_angle = fl_angle
          is_turn = is_turn and (self.far_left_angle == 0.0)
    else:
          self.far_left_angle = 0.0

    if far_right.size > 0:
          
          fr_angle = self.get_perLine_angle(self.get_straight_Line(far_right))
          self.far_right_angle = fr_angle
          is_turn = is_turn and (self.far_right_angle == 0.0)  
    else:
          self.far_right_angle = 0.0

    if is_turn:
        self.straight_line_count += 1
    else:
        self.straight_line_count = 0



  # Function to send the velocity to the robot
  def send_velocity(self, omega): 

    # omega = min(max(omega, -0.5), 0.5)
    if omega > 0.5:
        omega = 0.5
    elif omega < -0.5:
        omega = -0.5

    msg = Twist()
    msg.linear.x = self.forward_speed
    msg.angular.z = omega
    self.cmd_vel_publisher.publish(msg)
  

  ## Draw a blue overlay line on the image
  def draw_line(self, image, line):
    if line is not None:
      x1, y1, x2, y2 = line
      cv2.line(image, (x1, y1), (x2, y2), (255, 0, 0), 2)
        
  # Show the image
  def show_image(self, name, image):
    cv2.imshow(name, image)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'): # Pressing 'q' will shutdown the program
      rospy.signal_shutdown("exit")
    if key == ord('s'): # Pressing 's' will save the image
      cv2.imwrite("lane_follower_image.png", image)



  # # Calculate the rotational velocity
  # # width of the lane, mean left line, and mean right line
  # def calculate_omega(self, mean_left, mean_right, width):
    
  #   centerLane = (mean_left+mean_right)/2
  #   error = width - centerLane
  #   kp = 0.01
  #   omega = 0

  #   if self.forward_speed == 1.2:
  #     if error > 100 or error < -100:
  #         kp = 0.007
  #     else:
  #         kp = 0.01
  #   elif self.forward_speed >= 0.7:
  #     if error > 100 or error < -100:
  #       kp = 0.007
  #     else:
  #       kp = 0.035
  #   # else:
  #   #   # Exponential decay parameters
  #   #   C = 0.033  # initial kp value
  #   #   D = 0.01  # decay rate
  #   #   # Calculate the dynamic kp value based on the error 
  #   #   kp = C * np.exp(-D * abs(error))

  #   omega = (self.forward_speed * kp) * error
  #   return omega
    



  # Calculate the rotational velocity
  def calculate_omega(self, mean_left, mean_right, width):

    centerLane = (mean_left+mean_right)/2
    error = width - centerLane
    kp = 0.008
    omega = (self.forward_speed * kp) * error

    return omega


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
    
    ## Convert and filter HSV (use the calibrator to find correct HSV filter values)
    hsv_image = self.to_hsv(warped_image)
    filtered_hsv_image = self.filter_hsv(hsv_image, self.hsv_lower_values, self.hsv_upper_values)

    ## Filter the lines to determine which lane they belong to
    far_left, left, right, far_right = self.get_lines(filtered_hsv_image)

    ############################################################################################################
    ######################## Section is related to drawing the lines and the image #############################
    ############################################################################################################
    ## Make a copy of the warped image to draw the lines on
    warped_image_copy = warped_image.copy()
          
    # draw the lines on the warped image copy
    # non_empty_lines = [arr for arr in [far_left, left, right, far_right] if arr.any()]
    # concatenated_lines = np.concatenate(non_empty_lines) if non_empty_lines else np.array([])

    # Iterate through the concatenated array and draw lines
    # for line in concatenated_lines: self.draw_line(warped_image_copy, line)

    ## showing the warped image
    # self.show_image("warped_image_copy", warped_image_copy)
    ############################################################################################################
    # Determine the angles of each line
    self.get_angles(far_left, left, right, far_right)

    ## Determine the mean x value for the left, right, far_left and far_right line 
    mean_left, mean_right, mean_far_left, mean_far_right = None, None, None, None
    if left.any():
        mean_left = np.mean((left[:, 0] + left[:, 2]) / 2)
    if right.any():
        mean_right = np.mean((right[:, 0] + right[:, 2]) / 2)
    if far_left.any():
        mean_far_left = np.mean((far_left[:, 0] + far_left[:, 2]) / 2)
    if far_right.any():
        mean_far_right = np.mean((far_right[:, 0] + far_right[:, 2]) / 2)
    

    # calculating omega
    omega = self.calculate_omega(mean_left, mean_right, middle)

    # check if a straight path is detected
    self.straight_path = True if self.straight_line_count >= 10 else False
    # check if left lane exist
    self.left_lane_exist = True if mean_far_left is not None else False
    # check if right lane exist
    self.right_lane_exist = True if mean_far_right is not None else False


    # Check either lane if it exist
    if self.switch_left == True:
      if (not (self.left_lane_exist == False and self.right_lane_exist == False)):
        if self.left_angle == 0.0 and self.right_angle == 0.0 and self.far_left_angle == 0.0:
            if self.left_lane_exist == True:
                    omega = self.calculate_omega(mean_far_left, mean_left, middle)
            else:
                if self.right_lane_exist == True:
                    if self.left_sign == False:
                        # print("Turned")
                        self.switch_left = False
                        self.far_right_angle = 0.0

      # Make the turn to the right lane
    if self.switch_right == True:
      if (not (self.left_lane_exist == False and self.right_lane_exist == False)):
          if self.left_angle == 0.0 and self.right_angle == 0.0 and self.far_right_angle == 0.0:
            if self.right_lane_exist == True:
                omega = self.calculate_omega(mean_right, mean_far_right, middle)
            else:
                if self.left_lane_exist == True:
                    if self.right_sign == False:
                        # print("Turned")
                        self.switch_right = False
                        self.far_left_angle = 0.0
            


    
    # Send velocity
    self.send_velocity(omega)


   