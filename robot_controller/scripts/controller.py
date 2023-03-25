#! /usr/bin/env python
import rospy 
from lane_follower import LaneFollower
from laser_data import LaserData
from sign_recognizer import SignRecognizer
from object_tracker import ObjectTracker
from std_srvs.srv import Empty, EmptyResponse
from bot_message.srv import SetSpeed, SetSpeedResponse

class Controller(object):

  def __init__(self):
    self.forward_speed = 1.2 # Set the starting forward speed, DO NOT CHANGE
    self.lane_follower = LaneFollower(self.forward_speed)
    self.object_tracker = ObjectTracker()
    self.laser_data = LaserData(self.laser_callback)

    # temprery speed for the lane follower
    self.old_speed = 0.0
    
    # Change the parameter depending on if you are collecting data or using a trained network to predict. 
    # self.sign_recognizer = SignRecognizer(data_collection_mode = False, data_filename = "training")
    self.sign_recognizer = SignRecognizer(data_collection_mode = True, data_filename = "training") 

    # Service definitions
    self.switch_left_service = rospy.Service("/switch_left", Empty, self.initiate_switch_left)
    self.switch_right_service = rospy.Service("/switch_right", Empty, self.initiate_switch_right)
    self.set_speed_service = rospy.Service("/set_speed", SetSpeed, self.set_speed_callback)

  # Command via terminal to call this function: rosservice call /set_speed "speed: 0.0"
  def set_speed_callback(self, speed_msg):
    # Max forward speed should be 1.2 m/s
    # Update forward speed
    # Update lane follower forward speed 
    
    speed = round(speed_msg.speed, 1) # round to 1 decimal place
    if speed == 0.7:
        self.forward_speed = 0.7
        self.lane_follower.forward_speed = 0.7
    elif speed == 1.2:
        self.forward_speed = 1.2
        self.lane_follower.forward_speed = 1.2
    elif speed == 0.0:
        self.forward_speed = 0.0
        self.lane_follower.forward_speed = 0.0
    else:
        # reject any other speed values
        pass
    return SetSpeedResponse(speed_msg)
  
  # Command via terminal to call this function: rosservice call /switch_left
  def initiate_switch_left(self, empty_msg):
    # Check line angles to see if you can switch lanes
    # Start the lane switching
    self.lane_follower.switch_left = True

    return EmptyResponse()
  
  # Command via terminal to call this function: rosservice call /switch_right
  def initiate_switch_right(self, empty_msg):
    # Check line angles to see if you can switch lanes
    # Start the lane switching
    self.lane_follower.switch_right = True

    return EmptyResponse()
  

  # Laser callback function, gets called at 10Hz
  def laser_callback(self, laser_msg):
    # Get the laser data in cartesian coordinates
    data = self.laser_data.convert_to_cartesian(laser_msg)

    
    if data:
        cluster = self.laser_data.cluster(data)
        self.object_tracker.check_lanes(cluster)  

        # if object on the left lane
        if self.lane_follower.left_lane_exist:         
          if self.object_tracker.lane_occupied["left_lane"]:
              self.lane_follower.left_onhold = True
          else:
              self.lane_follower.left_onhold = False
        else:
              if not self.object_tracker.lane_occupied["left_lane"]:
                  self.lane_follower.left_onhold = False

        # if object on the right lane
        if self.lane_follower.right_lane_exist:   
          if self.object_tracker.lane_occupied["right_lane"]:
              self.lane_follower.right_onhold = True
          else:
              self.lane_follower.right_onhold = False
        else:
              if not self.object_tracker.lane_occupied["right_lane"]:
                  self.lane_follower.right_onhold = False

        # if object in front
        if self.object_tracker.lane_occupied["front"]:
          self.lane_follower.switch_left = True
          self.lane_follower.switch_right = True
          if (not self.lane_follower.left_lane_exist and not self.lane_follower.right_lane_exist) or (self.object_tracker.lane_occupied["left_lane"] or self.object_tracker.lane_occupied["right_lane"]) or (not self.lane_follower.straight_path):
              
              if self.old_speed == 0.0:
                  self.old_speed = self.lane_follower.forward_speed
                  self.lane_follower.forward_speed = 0.7   
        else:
           # nothing in front
           if self.lane_follower.switch_left == False and self.lane_follower.switch_right == False and self.old_speed != 0.0:

              self.lane_follower.forward_speed = self.old_speed
              self.old_speed = 0.0   

    self.signs_detection_callback()      

  # call the sign recognizer callback function
  def signs_detection_callback(self):
    # call the sign recognizer callback function and semd the orignal image
    self.sign_recognizer.classify(self.lane_follower.original_image)



   
if __name__ == "__main__":
      rospy.init_node("controller")
      controller = Controller()
      rospy.spin()



    # lane_follower: Call back
    # --------------------------
    # switch is True
    # Check if the far_line is not None
    # Check if all lines are is straight for 10 frames
    #
    # lane_follower: lane_switching
    # --------------------------
    # check if the left angle and the far_angle is 0.0 
    #           (check to assure omega will not be altered during the turn)
    # check if the mean_far and mean is not None 
  #             (A check related to extended switching time in case no straight line is detected)
    # check if its on hold
    # centerLane = !!!!!!!!!!(mean_left+mean_right)/2!!!!!!!!!! doiuble check this