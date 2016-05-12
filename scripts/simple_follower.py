#!/usr/bin/env python

import rospy
import numpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D

# a global variable for desired distance
ddist = 1.0
kp = 1.0


def human_track_cb(msg):
    d = numpy.sqrt(msg.x**2 + msg.y**2)
    delta = d - ddist

    cmd_vel = Twist()
    cmd_vel.linear.x = kp * delta
    cmd_vel.angular.z = -2 * kp * msg.x / d

    print msg
    print cmd_vel
    # publish the velocity
    vel_pub.publish(cmd_vel)


if __name__ == "__main__":
    rospy.init_node("simple_follower")
    pos_sub = rospy.Subscriber("human_pos2d", Pose2D, human_track_cb)
    vel_pub = rospy.Publisher("cmd_vel_mux/input/teleop", Twist, queue_size=1)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()
