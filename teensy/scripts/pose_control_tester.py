#!/usr/bin/python

from time import time
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
import matplotlib.pyplot as plt 


class PoseControlPlotter:
    def __init__(self):
        self.record = False

        self.x_data = []
        self.y_data = []
        self.yaw_data = []
        self.time_stamps = []
        self.start_time = 0
        self.last_time = 0
        self.x_cmd = 0
        self.y_cmd = 0
        self.yaw_cmd = 0
        self.x_sub = rospy.Subscriber("pose_control_pose", Pose2D, callback=self.pose_cb)
        self.cmd_sub = rospy.Subscriber("pos_cmd", Pose2D, callback=self.cmd_cb)

        self.stop_check_timer = rospy.Timer(rospy.Duration(1.0 / 10),callback=self.stop_check)

    def pose_cb(self, msg):
        if self.record:
            self.x_data += [msg.x]
            self.y_data += [msg.y]
            self.yaw_data += [msg.theta]
            self.time_stamps += [rospy.Time.now().to_sec() - self.start_time]
            self.last_time = rospy.Time.now().to_sec()
            #print(last_time)


    def cmd_cb(self, msg):
        self.x_data = []
        self.y_data = []
        self.yaw_data = []
        self.time_stamps = []
        self.record = True
        self.last_time = rospy.Time.now().to_sec()
        self.start_time = self.last_time
        self.x_cmd = msg.x
        self.y_cmd = msg.y
        self.yaw_cmd = msg.theta

        

    def plot_data(self):
        
        

        fig, (ax1, ax2, ax3) = plt.subplots(3,1, figsize = (10.24, 7.68))
        fig.suptitle("Pose control trajectory",fontsize=16)
        fig.subplots_adjust(hspace=0.5)
        ax1.plot(self.time_stamps, self.x_data)
        ax1.set_xlabel("Time (s)")
        ax1.set_ylabel("x position (m)")
       # ax1.title("x")

        ax2.plot(self.time_stamps, self.y_data)
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("y position (m)")
        #ax2.title("y")

        ax3.plot(self.time_stamps, self.yaw_data)
        ax3.set_xlabel("Time (s)")
        ax3.set_ylabel("theta (rad)")
        #ax3.title("theta")
        plt.savefig("pose_control_data.png")
        plt.show()

    def stop_check(self, asd):
        
        
        if self.record == True:
            if rospy.Time.now().to_sec() - self.last_time > 1.0:
                self.record = False
                self.plot_data()


if __name__ == "__main__":
    rospy.init_node("pose_control_tester")
    tester = PoseControlPlotter()
    rospy.spin()

