#!/usr/bin/env python

import rospy
import numpy as np
from smach import State, StateMachine
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from ford_project.msg import haptic_msg
from std_msgs.msg import String
from std_msgs.msg import Bool

# some global variables
ddist = 1.0
kp = 1.0
pitch_to_linear_scale = 0.8
yaw_to_angular_scale = -1.0

human_pos = Pose2D()
pos_status = "Lost"
cmd_button = Bool()     # button
cmd_rot = Vector3()     # roll, pitch, yaw

# a global publisher
vel_pub = rospy.Publisher("cmd_vel_mux/input/teleop", Twist, queue_size=1)


def human_track_pose_cb(msg):
    global human_pos
    human_pos = msg


def human_track_status_cb(msg):
    global pos_status
    pos_status = msg.data


def human_cmd_button_cb(msg):
    global cmd_button
    cmd_button = msg.data


def human_cmd_rot_cb(msg):
    global cmd_rot
    cmd_rot = msg


class Idle(State):
    def __init__(self):
        State.__init__(self, outcomes=["start_following", "cannot_start"])

    def execute(self, ud):
        rospy.loginfo("Executing state Idle")

        # wait for key input
        cmd = input("please press 's' to start following:")
        print cmd
        if cmd == 's':
            # wait for ready
            while pos_status != "Find":
                pass
            return "start_following"
        else:
            return "cannot_start"


class Following(State):
    def __init__(self):
        State.__init__(self, outcomes=["following_failed", "stop_following"])

    def execute(self, ud):
        rospy.loginfo("Executing state Following")

        # send following command to robot at 20Hz
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if pos_status == "Find":
                # calculate desired command
                d = np.sqrt(human_pos.x ** 2 + human_pos.y ** 2)
                delta = d - ddist

                cmd_vel = Twist()
                cmd_vel.linear.x = kp * delta
                cmd_vel.angular.z = -2 * kp * human_pos.x / d

                vel_pub.publish(cmd_vel)
            else:
                return "following_failed"

            rate.sleep()


class Pausing(State):
    def __init__(self):
        State.__init__(self, outcomes=["finished"])

    def execute(self, ud):
        rospy.loginfo("Executing state pausing")

        # send 0 velocity command and wait
        cmd_vel = Twist()
        vel_pub.publish(cmd_vel)

        rospy.sleep(1.0)

        return "finished"


class SendFB(State):
    def __init__(self):
        State.__init__(self, outcomes=["send_success", "send_failed"])

    def execute(self, ud):
        rospy.loginfo("Executing state send feedback")

        # do nothing for now, directly go to wait state
        return "send_success"


class WaitCMD(State):
    def __init__(self):
        State.__init__(self, outcomes=["wait_too_long", "cmd_received"])

    def execute(self, ud):
        rospy.loginfo("Executing state wait for command")

        # wait for the button signal
        rate = rospy.Rate(100)
        t_start = rospy.get_time()

        while rospy.get_time() - t_start < 5.0:
            if cmd_button:
                return "cmd_received"
            rate.sleep()

        # waited too long
        return "wait_too_long"


class Teleoperation(State):
    def __init__(self):
        State.__init__(self, outcomes=["restart_follow"])

    def execute(self, ud):
        rospy.loginfo("Executing state teleoperation")

        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            if cmd_button:
                # linear and angular velocity is proportional to pitch and yaw
                cmd_vel = Twist()
                # add in some dead-band
                if np.abs(cmd_rot.x) > 0.1:
                    cmd_vel.linear.x = cmd_rot.x / pitch_to_linear_scale
                if np.abs(cmd_rot.y) > 0.2:
                    cmd_vel.angular.z = cmd_rot.y / yaw_to_angular_scale
            else:
                cmd_vel = Twist()

            vel_pub.publish(cmd_vel)
            print cmd_vel

            # see if regain vision of human
            if pos_status == "Find":
                return "restart_follow"

            rate.sleep()


if __name__ == "__main__":
    # init the node
    rospy.init_node("simple_follower")

    # define subscribers and publishers
    pos_sub = rospy.Subscriber("tracking/human_pos2d", Pose2D, human_track_pose_cb)
    status_sub = rospy.Subscriber("tracking/status", String, human_track_status_cb)
    # subscriber for IMU input
    cmd_button_sub = rospy.Subscriber("human_input/button", Bool, human_cmd_button_cb)
    cmd_rot_sub = rospy.Subscriber("human_input/rotation", Vector3, human_cmd_rot_cb)

    sm = StateMachine(outcomes=["stopped"])

    with sm:
        StateMachine.add("IDLE", Idle(),
                         transitions={"start_following": "FOLLOWING",
                                      "cannot_start": "TELEOPERATION"})
        StateMachine.add("FOLLOWING", Following(),
                         transitions={"following_failed": "PAUSING",
                                      "stop_following": "IDLE"})
        StateMachine.add("PAUSING", Pausing(),
                         transitions={"finished": "SEND_FB"})
        StateMachine.add("SEND_FB", SendFB(),
                         transitions={"send_success": "WAIT_CMD",
                                      "send_failed": "SEND_FB"})
        StateMachine.add("WAIT_CMD", WaitCMD(),
                         transitions={"wait_too_long": "SEND_FB",
                                      "cmd_received": "TELEOPERATION"})
        StateMachine.add("TELEOPERATION", Teleoperation(),
                         transitions={"restart_follow": "FOLLOWING"})

    sm.execute()
