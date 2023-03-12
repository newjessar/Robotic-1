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
  
  # Get all the lines form the HSV image, and separate the lines into far_left_line, 
  # left_line, right_line, far_right_line
  def get_lines(self, binary_mask):
        
    _, binary_mask_width = binary_mask.shape
    middle_lane = binary_mask_width / 2
    segmented_width = middle_lane / 2

    # # Declare the lines
    far_left_line, left_line, right_line, far_right_line = [], [], [], []
    # Find all the lines using cv2.HoughLinesP function
    lines = np.asarray(cv2.HoughLinesP(binary_mask, 0.75, np.pi/180, 100, 100, 10))
   
    # Filter the lines into the four lists
    if lines.dtype == np.int32:
      for line in lines:
        x1, y1, x2, y2 = line[0]
        xAxis = np.mean([x1, x2])

        if xAxis > 0 and xAxis < middle_lane-segmented_width:
              far_left_line.append(line[0])
        elif xAxis > middle_lane-segmented_width and xAxis < middle_lane:
              left_line.append(line[0])
        elif xAxis > middle_lane and xAxis < middle_lane + segmented_width:
              right_line.append(line[0])
        elif xAxis > middle_lane + segmented_width:
              far_right_line.append(line[0])


    return np.asarray(far_left_line), np.asarray(left_line), np.asarray(right_line), np.asarray(far_right_line)
    
  ## Ge the highest and lowest x and y values for a given line
  def get_straight_Line(self, line):
    if line.size > 0:
      # straight_line = np.array([np.max(line[:, 0]), np.max(line[:, 1]), np.min(line[:, 2]), np.min(line[:, 3])])
      # new_line2 = np.array([233, 299, 233, 0]) # Ideal straight line
      straight_line = np.array([int(np.mean(line[:, 0])), np.max(line[:, 1]), int(np.mean(line[:, 2])), np.min(line[:, 3])])
      # print (">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
      # print("line: ", line)
      # print("straight_line:", straight_line)
      # print (">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")

      return straight_line

  ## Get the angle for a given line
  def get_perLine_angle(self, line):
      if line.size > 0:
            angle = np.arctan2(abs(line[0]) - abs(line[2]), abs(line[1]) - abs(line[3]))
            return angle

  # Get the angle for each line
  def get_angles(self, far_left, left, right, far_right):
    # print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    if left.size > 0:
          # self.left_angle = self.get_perLine_angle(left[0])
          self.left_angle = self.get_perLine_angle(self.get_straight_Line(left))
          # print("left angle: ", self.left_angle)
    else:
          self.left_angle = 0.0
    
    if right.size > 0:
          self.right_angle = self.get_perLine_angle(self.get_straight_Line(right))
          # print("right angle: ", self.right_angle)
    else:
          self.right_angle = 0.0
    
    if far_left.size > 0:
          self.far_left_angle = self.get_perLine_angle(self.get_straight_Line(far_left))
          # print("far left angle: ", self.far_left_angle)
    else:
          self.far_left_angle = 0.0

    if far_right.size > 0:
          self.far_right_angle = self.get_perLine_angle(self.get_straight_Line(far_right))
          # print("far right angle: ", self.far_right_angle)
    else:
          self.far_right_angle = 0.0
    # print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

  
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
    # mean_left and mean_right are the mean x values for the left_line and right_line, 
    # width is the (optional) width of the image, in case you want to normalize the error

    # Calculate omega with (self.forward_speed * Kp) * error
    # You need to define the Kp and error yourself.
    # Limit the omega to be between -0.5 and 0.5 rad/s.
    
    # proportional_controller
    centerLane = (mean_left+mean_right)/2
    error = width - centerLane
    kp = 0
    omega = 0

    if self.forward_speed == 1.2:
      if error > 100 or error < -100:
        kp = 0.00602 
    elif self.forward_speed >= 0.7 and self.forward_speed <= 0.8:
        kp =  0.015
    elif self.forward_speed >= 0.9 and self.forward_speed <= 1.0:
        kp =   0.0095
    elif self.forward_speed == 1.1:
        kp =   0.007
    elif self.forward_speed == 0.1:
          kp = 0.3
    elif self.forward_speed == 0.2:
          kp = 0.2
    elif self.forward_speed == 0.3:
          kp = 0.08
    elif self.forward_speed == 0.4:
          kp = 0.05
    elif self.forward_speed == 0.5:
          kp = 0.028
    elif self.forward_speed == 0.6:
          kp = 0.018


    # print("speed", self.forward_speed, "kp", kp, "error: ", error)
    omega = (self.forward_speed * kp) * error
    # print("omega", omega)
    return omega
    
    # (1) the robot is currently not in a turn and 
    # (2) there is another lane to switch to.
    # The function must also check the bool value self.switching_left and switching_right if its true 
    # The function must also check if the angles corrspond to the line that is being switched to must be 0.0
    # after the switch the function must set the self.switching_left and switching_right to false
    # in case the swtiiching is not possible then there is no need to do anything
  def lane_switcher(self, mean, mean_far, width):
    
    if self.switch_left:
        print("left_angle: ", self.left_angle, "far_left_angle: ", self.far_left_angle)
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
    # print("forward speed: ", self.forward_speed)

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

    # ## Draw the Probabilistic Line Transform. 
    # for line in left:
    #       self.draw_line(warped_image_copy, line)
    # for line in far_left:
    #       self.draw_line(warped_image_copy, line)
    # for line in right:
    #       self.draw_line(warped_image_copy, line)
    # for line in far_right:
    #       self.draw_line(warped_image_copy, line)
    
    
    ## Get a stripe of single straight line
    self.draw_line(warped_image_copy, self.get_straight_Line(left))
    self.draw_line(warped_image_copy, self.get_straight_Line(right))
    self.draw_line(warped_image_copy, self.get_straight_Line(far_left))
    self.draw_line(warped_image_copy, self.get_straight_Line(far_right))

    ## showing the warped image
    self.show_image("filtered_hsv_image", warped_image_copy)

    ## Determine the angles of each line
    self.get_angles(far_left, left, right, far_right)
    

    ## Determine the mean x value for the left, right, far_left and far_right line 
    mean_left, mean_right, mean_far_left, mean_far_right = None, None, None, None
    if left.any():
        mean_left = int(np.mean((self.get_straight_Line(left))[[0, 2]]))
    else:
        mean_left = None
    if right.any():
        mean_right = int(np.mean((self.get_straight_Line(right))[[0, 2]]))
    else:
        mean_right = None
    if far_left.any():
        mean_far_left = int(np.mean((self.get_straight_Line(far_left))[[0, 2]]))
    else:
        mean_far_left = None
    if far_right.any():
        mean_far_right = int(np.mean((self.get_straight_Line(far_right))[[0, 2]]))
    else:
        mean_far_right = None

    omega = 0.0
    ## calculate the omega
    if mean_left != None and mean_right != None:
        omega = self.calculate_omega(mean_left, mean_right, middle)
    
    elif mean_left == None and mean_right != None:
        omega = self.calculate_omega(0, mean_right, middle)   
          
    elif mean_left != None and mean_right == None:
        omega = self.calculate_omega(mean_left, 0, middle)

    # print("mean_left: ", mean_left, "mean_right: ", mean_right, "mean_far_left: ", mean_far_left, "mean_far_right: ", mean_far_right)
    # print("left_angle: ", self.left_angle, "right_angle: ", self.right_angle, "far_left_angle: ", self.far_left_angle, "far_right_angle: ", self.far_right_angle)
    
    ## Lane switching
    if self.switch_left == True:
      print("%%%%%%%%%%%%%%%%%%%%%%%%%%   Switch Left ")
      if mean_far_left != None:
          if(self.left_angle == 0.0 and self.right_angle == 0.0 and self.far_right_angle == 0.0):
              print("######################## First ########################")    
              print("mean_left: ", mean_left, "mean_right: ", mean_right, "mean_far_left: ", mean_far_left, "mean_far_right: ", mean_far_right)
              print("left_angle: ", self.left_angle, "right_angle: ", self.right_angle, "far_left_angle: ", self.far_left_angle, "far_right_angle: ", self.far_right_angle)
              print("Omega >>>>>>>>>>>>>> mean_left: ", mean_left, "mean_far_left: ", mean_far_left)
              omega = self.lane_switcher(mean_left, mean_far_left, middle)
      else:
          if mean_far_right != None:
            print("+++++++++++++++++++++++++++++++++   Else: left   ++++++++++++++++++++++++++++++++++++++++++++++")
            self.switch_left = False
            self.far_right_angle = 0.0


    if self.switch_right == True:
      print("%%%%%%%%%%%%%%%%%%%%%%%%%%   Switch right ")
      if mean_far_right != None:
          if(self.left_angle == 0.0 and self.right_angle == 0.0 and self.far_left_angle == 0.0):
              print("+++++++++++++++++++++++++++++++++   Second   ++++++++++++++++++++++++++++++++++++++++++++++")
              print("mean_left: ", mean_left, "mean_right: ", mean_right, "mean_far_left: ", mean_far_left, "mean_far_right: ", mean_far_right)
              print("left_angle: ", self.left_angle, "right_angle: ", self.right_angle, "far_left_angle: ", self.far_left_angle, "far_right_angle: ", self.far_right_angle)
              omega = self.lane_switcher(mean_right, mean_far_right, middle)
      else:
          if mean_far_left != None:
            print("+++++++++++++++++++++++++++++++++   Else: right   ++++++++++++++++++++++++++++++++++++++++++++++")
            self.switch_right = False
            self.far_left_angle = 0.0
    # else:
    #       print("--------------------------------------------------- No turn")

    if omega == None:
          omega = 0.0

    if omega == -0.0:
          omega = 0.0

    # print("omega Down: ", omega)
    # Send the velocity command
    self.send_velocity(omega)





    # Send velocity


   