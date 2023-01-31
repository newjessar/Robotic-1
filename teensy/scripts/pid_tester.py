#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import time
import matplotlib.pyplot as plt 

class PIDTester:
    def __init__(self):
        self.velocity_trajectory = []
        self.omega_trajectory = []
        self.dt = 0.02

        self.record = False
        self.start_time = 0
        self.left_targets = []
        self.left_target_time_stamps = []
        self.left_vels = []
        self.left_vel_time_stamps = []

        self.right_targets = []
        self.right_target_time_stamps = []
        self.right_vels = []
        self.right_vel_time_stamps = []

        self.left_target_sub = rospy.Subscriber("/left_target_velocity", Float32, callback=self.left_target_cb)
        self.left_wheel_vel_sub = rospy.Subscriber("/left_motor", Float32, callback=self.left_wheel_vel_cb)
        self.right_target_sub = rospy.Subscriber("/right_target_velocity", Float32, callback=self.right_target_cb)
        self.right_wheel_vel_sub = rospy.Subscriber("/right_motor", Float32, callback=self.right_wheel_vel_cb)

    def create_trajectories(self):
        acc = 0.3
        vel = 0
        omega = 0
        ang_acc = 1.2

        ## start with one second of zero velocities
        t = 0
        while t < 1.0:
            self.velocity_trajectory += [0]
            self.omega_trajectory += [0]
            t +=  self.dt

        ## accelerate forwards to 0.5 m/s
        while vel <= 0.5:
            vel += acc *  self.dt
            self.velocity_trajectory += [vel]
            self.omega_trajectory += [0]
            
        ## hold 0.5 m/s for 1 second
        t = 0
        while t < 1.0:
            self.velocity_trajectory += [vel]
            self.omega_trajectory += [0]
            t +=  self.dt
        ## stop and wait 1 second
        vel = 0
        omega = 0
        t = 0
        while t < 1.0:
            self.velocity_trajectory += [0]
            self.omega_trajectory += [0]
            t +=  self.dt

        ## accelerate backwards to -0.5 m/s
        while vel >= -0.5:
            vel -= acc *  self.dt
            self.velocity_trajectory += [vel]
            self.omega_trajectory += [0]

        ## hold -0.5 m/s for 1 second
        t = 0
        while t < 1.0:
            self.velocity_trajectory += [vel]
            self.omega_trajectory += [0]
            t +=  self.dt

        ## stop and wait 1 second
        t = 0
        while t < 1.0:
            self.velocity_trajectory += [0]
            self.omega_trajectory += [0]
            t +=  self.dt
        vel = 0
        omega = 0

        ## left angular acceleration until 3.14 rad/s
        while omega < 3.14:
            omega += ang_acc *  self.dt
            self.velocity_trajectory += [0]
            self.omega_trajectory += [omega]

        ## right angular acceleration until -3.14 rad/s
        while omega > -3.14:
            omega -= ang_acc *  self.dt
            self.velocity_trajectory += [0]
            self.omega_trajectory += [omega]

        ## left angular acceleration until stopped.
        while omega < 0:
            omega += ang_acc *  self.dt
            self.velocity_trajectory += [0]
            self.omega_trajectory += [omega]

        ## stop and wait 1 second
        t = 0
        while t < 1.0:
            self.velocity_trajectory += [0]
            self.omega_trajectory += [0]
            t +=  self.dt

    def plot(self):
        plt.figure(figsize = (10.24, 7.68))

        plt.plot(self.left_vel_time_stamps, self.left_vels, label="v_left")
        plt.plot(self.right_vel_time_stamps, self.right_vels, label= "v_right")
        plt.plot(self.right_target_time_stamps, self.right_targets, label="v_right_target")
        plt.plot(self.left_target_time_stamps, self.left_targets, label="v_left_target")
        plt.xlabel("time(s)")
        plt.ylabel("velocity (rad/s)")
        plt.title("PID profile")
        plt.legend()
        plt.savefig("pid_profile.png")
        plt.show()

    def run(self):
        pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        time.sleep(0.2)
        self.create_trajectories()
        print("created trajectories..")
        print(len(self.velocity_trajectory), len(self.omega_trajectory))
        print("total time = ", len(self.velocity_trajectory) * self.dt)
        self.record = True
        self.start_time = rospy.Time.now().to_sec()
        for idx in range(len(self.velocity_trajectory)):
            msg = Twist()
            msg.linear.x = self.velocity_trajectory[idx]
            msg.angular.z = self.omega_trajectory[idx]
            pub.publish(msg)
            time.sleep(self.dt)
        self.record = False
        self.plot()

    def left_target_cb(self, msg):
        if self.record:
            self.left_targets += [msg.data]
            self.left_target_time_stamps += [rospy.Time.now().to_sec() - self.start_time]

    def left_wheel_vel_cb(self, msg):
        if self.record:
            self.left_vels += [msg.data]
            self.left_vel_time_stamps += [rospy.Time.now().to_sec() - self.start_time]

    def right_target_cb(self, msg):
        if self.record:
            self.right_targets += [msg.data]
            self.right_target_time_stamps += [rospy.Time.now().to_sec() - self.start_time]

    def right_wheel_vel_cb(self, msg):
        if self.record:
            self.right_vels += [msg.data]
            self.right_vel_time_stamps += [rospy.Time.now().to_sec() - self.start_time]


if __name__ == "__main__":
    rospy.init_node("pid_velocity_tester")
    tester = PIDTester()

    tester.run()

