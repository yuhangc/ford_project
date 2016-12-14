#!/usr/bin/env python

import rospy
import numpy as np

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3


class VelocitySmoother:
    def __init__(self):
        # set parameters
        self.max_inc_lin = rospy.get_param("~maximum_increment_linear", 0.02)
        self.max_inc_rot = rospy.get_param("~maximum_increment_angular", 0.05)
        self.vel_cap_lin = rospy.get_param("~velocity_cap_linear", 1.0)
        self.vel_cap_rot = rospy.get_param("~velocity_cap_angular", 2.0)

        self.vel_now = Twist()
        self.vel_goal = Twist()

        # tolerance for velocity
        self.vel_tol_lin = 0.01
        self.vel_tol_rot = 0.01

        # subscribers and publishers
        self.cmd_vel_sub = rospy.Subscriber("cmd_vel",
                                            Twist, self.cmd_vel_callback)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel_smooth", Twist, queue_size=1)

    @staticmethod
    def cap_velocity(vel, vel_cap):
        if vel > vel_cap:
            vel = vel_cap
        elif vel < -vel_cap:
            vel = -vel_cap
        return vel

    def cmd_vel_callback(self, msg):
        self.vel_goal = msg

        # cap the goal velocity if necessary
        self.vel_goal.linear.x = self.cap_velocity(self.vel_goal.linear.x, self.vel_cap_lin)
        self.vel_goal.angular.z = self.cap_velocity(self.vel_goal.angular.z, self.vel_cap_rot)

    def update_vel(self):
        # publish the new velocity
        self.cmd_vel_pub.publish(self.vel_now)

        # do nothing if goal is reached
        flag_vel_lin = np.abs(self.vel_goal.linear.x - self.vel_now.linear.x) < self.vel_tol_lin
        flag_vel_rot = np.abs(self.vel_goal.angular.z - self.vel_now.angular.z) < self.vel_tol_rot

        if flag_vel_lin and flag_vel_rot:
            self.vel_now.linear.x = self.vel_goal.linear.x
            self.vel_now.angular.z = self.vel_goal.angular.z
            return

        # update the current velocity
        self.vel_now.linear.x += self.cap_velocity(self.vel_goal.linear.x - self.vel_now.linear.x,
                                                   self.max_inc_lin)
        self.vel_now.angular.z += self.cap_velocity(self.vel_goal.angular.z - self.vel_now.angular.z,
                                                    self.max_inc_rot)

if __name__ == "__main__":
    # initialize node
    rospy.init_node("velocity_smoother")

    # create a simple follower object
    vel_smoother = VelocitySmoother()

    # loop rate 100 Hz
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        vel_smoother.update_vel()
        rate.sleep()
