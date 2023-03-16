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

    self.switch_left = False
    self.switch_right = False

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
    flo_lines, width, prec, nfa = lsd.detect(binary_mask)

    # Round the coordinates to the nearest integer and convert them to integers
    lines = np.round(flo_lines).astype(np.int32)

   
    # Filter the lines into the four lists
    if lines.any():
      
        for line in lines:
              
          x1, y1, x2, y2 = line[0]
          xAxis = ((x1+ x2)/2)
          
          if xAxis > 0 and xAxis < middle_lane-segmented_width:
                far_left.append(line[0])
          elif xAxis > middle_lane-segmented_width and xAxis < middle_lane:
                left.append(line[0])
          elif xAxis > middle_lane and xAxis < middle_lane + segmented_width:
                right.append(line[0])
          elif xAxis > middle_lane + segmented_width:
                far_right.append(line[0])

    return np.asarray(far_left), np.asarray(left), np.asarray(right), np.asarray(far_right)
  
    
  ## Ge the highest and lowest x and y values for a given line
  def get_straight_Line(self, line):
    if line.size > 0:
      # Calculate the sum of x1 and x2 for each line
      x_sums = line[:, 0] + line[:, 2]
      # Get the lines with the minimum and maximum x sums
      min_line = line[np.argmin(x_sums)]
      max_line = line[np.argmax(x_sums)]

      # print("Line with maximum x1 + x2:", max_line)
      # print("Line with minimum x1 + x2:", min_line)

      return max_line

  ## Get the angle for a given line
  def get_perLine_angle(self, line):
      if line.size > 0:
            angle = atan2(abs(line[0]) - abs(line[2]), abs(line[1]) - abs(line[3]))
            # print("Angle:", angle)
            if angle < 0.0:
                return 0.0
            else:
              return angle

  # Get the angle for each line
  def get_angles(self, far_left, left, right, far_right):
    if left.size > 0:
          self.left_angle = self.get_perLine_angle(self.get_straight_Line(left))
    else:
          self.left_angle = 0.0
    
    if right.size > 0:
          self.right_angle = self.get_perLine_angle(self.get_straight_Line(right))
    else:
          self.right_angle = 0.0
    
    if far_left.size > 0:
          self.far_left_angle = self.get_perLine_angle(self.get_straight_Line(far_left))
    else:
          self.far_left_angle = 0.0

    if far_right.size > 0:
          self.far_right_angle = self.get_perLine_angle(self.get_straight_Line(far_right))
    else:
          self.far_right_angle = 0.0

  
  def send_velocity(self, omega): 
    # # Create a Twist message
    # # For the forward velocity use the self.forward_velocity variable
    # # use the self.cmd_vel_publisher to publish the velocity
    # # Limit the rotational velocity to (-)0.5 rad/s, this will help with switching lanes

    msg = Twist()
    msg.linear.x = self.forward_speed
    msg.angular.z = omega
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

  # Calculate the rotational velocity
  def calculate_omega(self, mean_left, mean_right, width):
    # mean_left and mean_right are the mean x values for the left and right, 
    # width is the (optional) width of the image, in case you want to normalize the error

    # Calculate omega with (self.forward_speed * Kp) * error
    # You need to define the Kp and error yourself.
    # Limit the omega to be between -0.5 and 0.5 rad/s.
    
    # proportional_controller
    centerLane = (mean_left+mean_right)/2
    error = width - centerLane
    kp = 0.01
    omega = 0


    if self.forward_speed == 1.2:
        kp = 0.01
    elif self.forward_speed >= 0.7:
        kp = 0.035


    omega = (self.forward_speed * kp) * error
    print("omega:", omega, "error:", error, "kp * forward_speed:", (self.forward_speed * kp))

    return omega
    
  # Lane switcher function
  def lane_switcher(self, mean, mean_far, width):
    
    if self.switch_left:
        if self.left_angle == 0.0 and self.far_left_angle == 0.0:
          if mean_far != None and mean != None:
            omega = self.calculate_omega(mean_far, mean, width)
            return omega
          
          elif self.left_angle == 0.0:
                self.switch_left = False
    
    if self.switch_right:
        if self.right_angle == 0.0 and self.far_right_angle == 0.0:
          if mean_far != None and mean != None:
            omega = self.calculate_omega(mean, mean_far, width)
            return omega

          elif self.right_angle == 0.0:
                self.switch_right = False
          
          


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
    segment_width = middle / 2 # Determine the width of each segment of the image
    
    ## Convert and filter HSV (use the calibrator to find correct HSV filter values)
    hsv_image = self.to_hsv(warped_image)
    filtered_hsv_image = self.filter_hsv(hsv_image, self.hsv_lower_values, self.hsv_upper_values)

    ## Filter the lines to determine which lane they belong to
    far_left, left, right, far_right = self.get_lines(filtered_hsv_image)


    ## Make a copy of the warped image to draw the lines on
    warped_image_copy = warped_image.copy()
          
          
    # draw the lines on the warped image copy
    if far_left.any():
          for line in far_left:
                self.draw_line(warped_image_copy, line)
    if left.any():
          for line in left:
                self.draw_line(warped_image_copy, line)
    if right.any():
          for line in right:
                self.draw_line(warped_image_copy, line)
    if far_right.any():
          for line in far_right:
                self.draw_line(warped_image_copy, line)
 
    
    ## showing the warped image
    self.show_image("warped_image_copy", warped_image_copy)

    # Determine the angles of each line
    self.get_angles(far_left, left, right, far_right)
    
    # # print("left line", left)
    # if self.left_angle != 0.0:
    #     print("left angle: ", self.left_angle)
    #     print("left line: ", left)
    # if self.right_angle != 0.0:
    #     print("right angle: ", self.right_angle)
    #     print("right line: ", right)
    # if self.far_left_angle != 0.0:
    #     print("far left angle: ", self.far_left_angle)
    #     print("far left line: ", far_left)
    # if self.far_right_angle != 0.0:
    #     print("far right angle: ", self.far_right_angle)
    #     print("far right line: ", far_right)

    print("---------------------------------")
    ## Determine the mean x value for the left, right, far_left and far_right line 
    # mean_left, mean_right, mean_far_left, mean_far_right = None, None, None, None
    # if left.any():
    #     mean_left = int(np.mean((self.get_straight_Line(left))[[0, 2]]))
    #     print("mean_left: ", mean_left)
    # else:
    #     mean_left = None
    # if right.any():
    #     mean_right = int(np.mean((self.get_straight_Line(right))[[0, 2]]))
    #     print("mean_right: ", mean_right)
    # else:
    #     mean_right = None
    # if far_left.any():
    #     mean_far_left = int(np.mean((self.get_straight_Line(far_left))[[0, 2]]))
    #     print("mean_far_left: ", mean_far_left)
    # else:
    #     mean_far_left = None
    # if far_right.any():
    #     mean_far_right = int(np.mean((self.get_straight_Line(far_right))[[0, 2]]))
    #     print("mean_far_right: ", mean_far_right)
    # else:
    #     mean_far_right = None
    # print("---------------------------------")
        # creating omega and the switch lane part
    if left.any():
        left_av_x = np.mean((left[:, 0] + left[:, 2]) / 2)

    if right.any():
        right_av_x = np.mean((right[:, 0] + right[:, 2]) / 2)

    # calculating omega
    omega = self.calculate_omega(left_av_x, right_av_x, middle)


    # # omega = 0.0
    # ## calculate the omega
    # if mean_left != None and mean_right != None:
    #       omega = self.calculate_omega(mean_left, mean_right, middle)
    
    # elif mean_left == None and mean_right != None:
    #       omega = self.calculate_omega(0, mean_right, middle)   
          
    # elif mean_left != None and mean_right == None:
    #     omega = self.calculate_omega(mean_left, 0, middle)


    # ## Lane switching
    # if self.switch_left == True:
    #   if mean_far_left != None:
    #       if(self.left_angle == 0.0 and self.right_angle == 0.0 and self.far_right_angle == 0.0):
    #           omega = self.lane_switcher(mean_left, mean_far_left, middle)
    #   else:
    #       if mean_far_right != None:
    #         self.switch_left = False
    #         self.far_right_angle = 0.0


    # if self.switch_right == True:
    #   if mean_far_right != None:
    #       if(self.left_angle == 0.0 and self.right_angle == 0.0 and self.far_left_angle == 0.0):
    #           omega = self.lane_switcher(mean_right, mean_far_right, middle)
    #   else:
    #       if mean_far_left != None:
    #         self.switch_right = False
    #         self.far_left_angle = 0.0

    # Send velocity
    self.send_velocity(omega)


   