#!/usr/bin/env python

import rospy
import numpy as np

from tf import transformations

from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from ford_project.msg import haptic_msg
from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Bool

states_all = {0: "Idle",
              1: "Follow",
              2: "LostVision",
              3: "GetStuck",
              4: "Teleop"}      # teleop state is only for testing

cmd_states_all = {0: "Idle",
                  1: "SendOnce",
                  2: "sendPeriod"}

input_mode_all = {0: "gesture_control",
                  1: "tilt_control"}

gesture_dict = {0: "forward",
                1: "backward",
                2: "turn_cw",
                3: "turn_ccw"}


class SimpleFollower:
    def __init__(self):
        # state for the state machine
        self.state = "Idle"
        self.cmd_state = "Idle"

        # timer for state machine
        self.cmd_state_timer = rospy.get_time()
        self.cmd_state_period = 0.0

        # desired velocity for publish
        self.cmd_vel = Twist()

        # human tracking variables
        self.human_pose = Pose2D()
        self.track_status = "Lost"

        # human input variables
        self.human_input_tilt = np.array([0.0, 0.0, 0.0])    # roll, pitch, yaw
        self.human_input_gesture = -1               # 10 means no gesture input
        self.human_input_mode = "gesture_control"

        # state machine control
        self.set_state = -1     # < 0 means no set state command
        self.set_cmd_state = -1

        # variables for follower control
        self.dist_desired = 1.0
        self.kp_follower = 1.0

        self.dist_range_min = 0.5
        self.dist_range_max = 3.0

        # variables for tilt control
        self.pitch_to_linear_scale = 1.0
        self.roll_to_angular_scale = -1.0
        self.pitch_deadband = 0.3
        self.roll_deadband = 0.3
        self.pitch_offset = 0.2
        self.roll_offset = 0.2

        # subscribers to human tracking
        self.human_pose_sub = rospy.Subscriber("tracking/human_pos2d",
                                               Pose2D, self.human_track_pose_cb)
        self.track_status_sub = rospy.Subscriber("tracking/status",
                                                 String, self.human_track_status_cb)
        # subscribers to human input
        self.human_input_ort_sub = rospy.Subscriber("human_input/tilt",
                                                    Quaternion, self.human_input_tilt_cb)
        self.human_input_gesture_sub = rospy.Subscriber("human_input/gesture",
                                                        Int8, self.human_input_gesture_cb)
        self.human_input_mode_sub = rospy.Subscriber("human_input/mode",
                                                     Bool, self.human_input_mode_cb)

        # subscriber to state control
        self.state_control_sub = rospy.Subscriber("state_control/set_state",
                                                  Int8, self.set_state_cb)
        self.cmd_state_control_sub = rospy.Subscriber("cmd_state_control/set_state",
                                                      Int8, self.set_cmd_state_cb)

        # publisher to robot velocity
        self.robot_vel_pub = rospy.Publisher("cmd_vel_mux/input/teleop",
                                             Twist, queue_size=1)

    # call back functions
    def human_track_pose_cb(self, msg):
        self.human_pose = msg

    def human_track_status_cb(self, msg):
        self.track_status = msg.data

    def human_input_tilt_cb(self, msg):
        q = np.array([msg.x, msg.y, msg.z, msg.w])
        self.human_input_tilt = transformations.euler_from_quaternion(q, axes='rxyz')
        rospy.loginfo("tilt angles are %f %f %f", self.human_input_tilt[0],
                      self.human_input_tilt[1], self.human_input_tilt[2])

    def human_input_gesture_cb(self, msg):
        self.human_input_gesture = msg.data

    def human_input_mode_cb(self, msg):
            self.human_input_mode = input_mode_all[msg.data]

    def set_state_cb(self, msg):
        self.set_state = msg.data
        rospy.loginfo("set state to be %s", states_all[self.set_state])

    def set_cmd_state_cb(self, msg):
        self.set_cmd_state = msg.data
        self.cmd_state = cmd_states_all[self.set_cmd_state]
        rospy.loginfo("set command state to %s", self.cmd_state)

    # utility functions
    def send_vel_cmd(self, vx, omg, T=0.0):
        # do nothing if state is not idle
        if self.cmd_state != "Idle":
            return

        # set and send desired velocity
        self.cmd_vel.linear.x = vx
        self.cmd_vel.angular.z = omg

        if T == 0:
            # set state to send vel once
            self.cmd_state = "SendOnce"
        else:
            self.cmd_state_period = T
            self.cmd_state_timer = rospy.get_time()
            self.cmd_state = "SendPeriod"

    def check_set_state(self):
        # check if state has been set to idle or teleop manually
        if self.set_state == 0:
            self.set_state = -1
            self.send_vel_cmd(0, 0)
            self.state = "Idle"
        elif self.set_state == 4:
            self.set_state = -1
            self.send_vel_cmd(0, 0)
            self.state = "Teleop"

    # state functions
    def idle(self):
        # check for start message
        if self.set_state == 1:
            # check if already have human in vision
            if self.track_status == "Lost":
                rospy.logwarn("Cannot start following! Human not found!")
            else:
                # set to state follow
                self.state = "Following"
        elif self.set_state == 4:
            # directly to go state teleop
            self.state = "Teleop"

    def follow(self):
        # check if need to switch state
        if self.track_status == "Lost":
            # set robot to stop and go to state lost vision
            self.send_vel_cmd(0, 0)
            self.state = "LostVision"
            return

        self.check_set_state()

        # calculate desired velocity to follow
        d = np.sqrt(self.human_pose.x ** 2 + self.human_pose.y ** 2)
        vx = self.kp_follower * (self.dist_desired - d)
        omg = -2.0 * self.kp_follower * self.human_pose.x / d

        self.send_vel_cmd(vx, omg)

    def lost_vision(self):
        # check if regains sight, and in proper range
        if self.track_status == "Find":
            d = np.sqrt(self.human_pose.x **2 + self.human_pose.y ** 2)
            if self.dist_range_min < d < self.dist_range_max:
                # go back to following
                self.state = "Follow"

        # do teleop
        self.teleop()

    def get_stuck(self):
        # check if the path is clear??
        # right now just wait for start follow signal
        if self.set_state == 1:
            if self.track_status == "Find":
                d = np.sqrt(self.human_pose.x ** 2 + self.human_pose.y ** 2)
                if self.dist_range_min < d < self.dist_range_max:
                    self.set_state = -1
                    self.state = "Follow"

        # do teleoperation
        self.teleop()

    def teleop(self):
        rospy.loginfo("in teleoperation")
        if self.human_input_mode == "gesture_control":
            if self.human_input_gesture < 0:
                gesture = "do_nothing"
            else:
                gesture = gesture_dict[self.human_input_gesture]

            if gesture == "forward":
                self.send_vel_cmd(0.5, 0, 0.5)
            elif gesture == "backward":
                self.send_vel_cmd(-0.5, 0, 0.5)
            elif gesture == "turn_cw":
                self.send_vel_cmd(0, -0.5, 0.5)
            elif gesture == "turn_ccw":
                self.send_vel_cmd(0, 0.5, 0.5)
            else:
                self.send_vel_cmd(0, 0)

            self.human_input_gesture = -1
        else:
            if self.human_input_tilt[0] > self.roll_deadband:
                omg = self.roll_to_angular_scale * (self.human_input_tilt[0] - self.roll_offset)
            elif self.human_input_tilt[0] < -self.roll_deadband:
                omg = self.roll_to_angular_scale * (self.human_input_tilt[0] + self.roll_offset)
            else:
                omg = 0

            if self.human_input_tilt[1] > self.pitch_deadband:
                vx = self.pitch_to_linear_scale * (self.human_input_tilt[1] - self.pitch_offset)
            elif self.human_input_tilt[1] < -self.pitch_deadband:
                vx = self.pitch_to_linear_scale * (self.human_input_tilt[1] + self.pitch_offset)
            else:
                vx = 0

            self.send_vel_cmd(vx, omg)
            rospy.loginfo("cmd_vel_sent!")

    def update(self):
        # run over states
        if self.state == "Idle":
            self.idle()
        elif self.state == "Follow":
            self.follow()
        elif self.state == "LostVision":
            self.lost_vision()
        elif self.state == "GetStuck":
            self.get_stuck()
        elif self.state == "Teleop":
            self.teleop()
        else:
            rospy.logerr("Unknown state!")

        # a mini state machine for sending desired velocity
        if self.cmd_state == "Idle":
            pass
        elif self.cmd_state == "SendOnce":
            # publish the velocity and go back to idle
            self.robot_vel_pub.publish(self.cmd_vel)
            self.cmd_state = "Idle"
        elif self.cmd_state == "SendPeriod":
            # keep publishing until timer is up
            self.robot_vel_pub.publish(self.cmd_vel)

            if rospy.get_time() - self.cmd_state_timer > self.cmd_state_period:
                self.cmd_state = "Idle"


if __name__ == "__main__":
    # initialize node
    rospy.init_node("simple_follower")

    # create a simple follower object
    follower = SimpleFollower()

    # loop rate 100 Hz
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        follower.update()
        rate.sleep()