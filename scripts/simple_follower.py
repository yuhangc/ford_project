#!/usr/bin/env python

import rospy
import numpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D


class SimpleFollower:
    def __init__(self):
        self.ddist = 1.0
        self.kp = 1.0

        # publishers and subscribers
        self.pos_sub = rospy.Subscriber("human_pos2d", Pose2D, self.human_track_cb)
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/teleop", Twist, queue_size=1)

    def human_track_cb(self, msg):
        d = numpy.sqrt(msg.x ** 2 + msg.y ** 2)
        delta = d - self.ddist

        cmd_vel = Twist()
        cmd_vel.linear.x = self.kp * delta
        cmd_vel.angular.z = -2 * self.kp * msg.x / d

        # print msg
        # print cmd_vel
        # publish the velocity
        self.vel_pub.publish(cmd_vel)


if __name__ == "__main__":
    rospy.init_node("simple_follower")

    follower = SimpleFollower()

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()
