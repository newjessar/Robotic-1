#!/usr/bin/python
import cv2
import rospy
from geometry_msgs.msg import Pose2D
import numpy as np
import time
class PoseVisualizer:

    def __init__(self):
        self.image = np.zeros((500,500,3), np.uint8)
        self.pose_publisher = rospy.Publisher("pos_cmd", Pose2D, queue_size=1)
        cv2.imshow("Position control interface", self.image)
        cv2.waitKey(50)
        self.x = 0
        self.y = 0
        self.theta = 0
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        self.pos_target_x = 0
        self.pos_target_y = 0
        self.pos_target_theta = 0
        self.update(0,0,0)

        self.target_subscriber = rospy.Subscriber("pos_cmd", Pose2D, self.target_callback)
        self.pose_subscriber = rospy.Subscriber("pose_control_pose", Pose2D, self.pose_callback)
        print("Position control interface")
        print("For sending pose commands, the following keybindings are supported by default:")
        print("\tW: Move forward (0.5, 0.0, 0.0)")
        print("\tA: Turn left (0.5, 0.5, 1.57)")
        print("\tD: Turn right (0.5, -0.5, -1.57)")
        print("\tSpace: stop (0,0,0)")

        
    def publish_cmd(self, x, y, theta):
        msg = Pose2D()
        msg.x = x
        msg.y = y
        msg.theta = theta
        self.pose_publisher.publish(msg)

    def target_callback(self, msg):
        self.pos_target_x = msg.x
        self.pos_target_y = msg.y
        self.pos_target_theta = msg.theta

    def timer_callback(self, asd):
        self.update(self.x, self.y, self.theta)
        
    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        

    def reset_background(self):
        self.image = np.zeros((500,500,3), dtype = np.uint8)
        self.image = cv2.line(self.image, (250, 250), (500, 250), (0,125,0))
        self.image = cv2.circle(self.image, (250,250), 250, (0,125,0))
        self.image = cv2.circle(self.image, (250,250), 125 + 68, (0,125,0))

        self.image = cv2.circle(self.image, (250,250), 125, (0,125,0))
        self.image = cv2.circle(self.image, (250,250), 68, (0,125,0))
        
    def meters_to_pixels(self, coord):
        return int(250 + coord * 250 / 2.0 )

    def update(self, x, y, theta):
        x_scaled = self.meters_to_pixels(x)
        y_scaled = 500 - self.meters_to_pixels(y)
        start = (x_scaled, y_scaled)
        length = 25
        end = (int(x_scaled + length * np.cos(theta)), int(y_scaled - length * np.sin(theta)))
        thickness = 1
        color = (0, 0, 255)
        self.reset_background()
        self.image = cv2.putText(self.image, "Current pose: (Target coordinate system)", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))
        self.image = cv2.putText(self.image, "x: " + str(x), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))
        self.image = cv2.putText(self.image, "y: " + str(y), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))
        self.image = cv2.putText(self.image, "theta: " + str(theta), (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))

        self.image = cv2.putText(self.image, "Target pose: (Robot coordinate system)", (10, 420), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))
        self.image = cv2.putText(self.image, "x: " + str(self.pos_target_x), (10, 440), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))
        self.image = cv2.putText(self.image, "y: " + str(self.pos_target_y), (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))
        self.image = cv2.putText(self.image, "theta: " + str(self.pos_target_theta), (10, 480), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))

        self.image = cv2.arrowedLine(self.image, start, end, color, thickness,tipLength = 0.2)
        cv2.imshow("Position control interface", self.image)
        key = cv2.waitKey(10)
        #print(key)
        if key ==  ord(' '):
            self.publish_cmd(0,0,0)
        elif key == ord('w'):
            self.publish_cmd(0.5, 0, 0)
        elif key == ord('a'):
            self.publish_cmd(0.5, 0.5, 1.57)
        elif key == ord('d'):
            self.publish_cmd(0.5, -0.5, -1.57)
        

if __name__ == "__main__":
    rospy.init_node("pose_visualizer")
    posevis = PoseVisualizer()
    time.sleep(2)
    rospy.spin()





