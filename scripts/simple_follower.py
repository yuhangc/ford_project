#!/usr/bin/env python

import rospy
import numpy as np

from tf import transformations

from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import Led
from kobuki_msgs.msg import DigitalOutput

from ford_project.msg import haptic_msg

states_all = {0: "Idle",
              1: "Follow",
              2: "LostVision",
              3: "GetStuck",
              4: "Teleop"}  # teleop state is only for testing

cmd_states_all = {0: "Idle",
                  1: "SendOnce",
                  2: "sendPeriod"}

input_mode_all = {1: "gesture_control",
                  0: "tilt_control"}

gesture_dict = {0: "twist",
                1: "right_cw",
                2: "forward",
                3: "backward",
                4: "start",
                5: "stop"}


class SimpleFollower:
    def __init__(self):
        # cv2.namedWindow("status", 1)

        # set initial states
        self.state = rospy.get_param("~state_init", "Idle")
        self.cmd_state = rospy.get_param("~cmd_state_init", "Idle")

        # flag that indicates whether haptic tether is enables
        # 0 - Teleoperation
        # 1 - Autonomous
        # 2 - Haptic Tether
        self.set_follower_mode = 2

        # timer for state machine
        self.cmd_state_timer = rospy.get_time()
        self.cmd_state_period = 0.0

        # loop counters
        self.stuck_count = 0
        self.stuck_backup_count = 0
        self.lost_vision_count = 0
        self.too_fast_count = 0

        self.stuck_backup_count_limit = 50
        self.stuck_count_limit = rospy.get_param("~stuck_count_limit", 25)
        self.lost_vision_count_limit = rospy.get_param("~lost_vision_count_limit", 25)
        self.too_fast_count_limit = rospy.get_param("~too_fast_count_limit", 25)
        self.turtlebot_vel_max = rospy.get_param("~turtlebot_vel_max", 0.7)

        # desired velocity for publish
        self.cmd_vel = Twist()
        self.measured_vel = Twist()

        # teleoperation parameters
        self.tele_vel = Twist()
        self.tele_vel.linear.x = rospy.get_param("~default_teleop_vel_linear", 0.3)
        self.tele_vel_inc_linear = rospy.get_param("~telop_vel_increment_linear", 0.01)

        # bumper variable
        self.bumper_event = BumperEvent()

        # human tracking variables
        self.human_pose = Pose2D()
        self.human_vel = Vector3()
        self.track_status = "Lost"

        # human input variables
        self.human_input_tilt = Vector3()  # roll, pitch, yaw
        self.human_input_gesture = -1  # 10 means no gesture input

        self.flag_button_pressed = False
        self.flag_latch_cmd_vel = False
        self.human_input_mode = rospy.get_param("~human_input_mode", "tilt_control")

        # state machine control
        self.set_state = -1  # < 0 means no set state command
        self.set_cmd_state = -1

        # variables for follower control
        self.dist_desired = rospy.get_param("~dist_desired_follower", 1.0)
        self.kp_linear = rospy.get_param("~kp_linear", 1.0)
        self.kd_linear = rospy.get_param("~kd_linear", 0.1)
        self.kb_linear = rospy.get_param("~kb_linear", 0.1)
        self.kp_angular = rospy.get_param("~kp_angular", 2.0)
        self.kd_angular = rospy.get_param("~kd_angular", 0.2)
        self.kb_angular = rospy.get_param("~kb_angular", 0.2)

        self.dist_range_min = rospy.get_param("~dist_range_min", 0.3)
        self.dist_range_max = rospy.get_param("~dist_range_max", 5.0)

        self.dist_stuck_resume_max = rospy.get_param("~dist_stuck_resume_max", 1.0)

        # variables for tilt control
        self.roll_to_linear_scale = rospy.get_param("~roll_to_linear_scale", 1.0)
        self.pitch_to_angular_scale = rospy.get_param("~pitch_to_angular_scale", -3.0)
        self.pitch_deadband = rospy.get_param("~pitch_deadband", 0.4)
        self.roll_deadband = rospy.get_param("~roll_deadband", 0.4)
        self.pitch_offset = rospy.get_param("pitch_offset", 0.2)
        self.roll_offset = rospy.get_param("roll_offset", 0.2)
        self.roll_center_offset = rospy.get_param("roll_center_offset", 0.1)

        # haptic signal thresholds
        self.haptic_dir_thresh = 30
        self.haptic_amp_thresh = 2.5

        # subscribers to human tracking
        self.human_pose_sub = rospy.Subscriber("tracking/human_pos2d",
                                               Pose2D, self.human_track_pose_cb)
        self.human_vel_sub = rospy.Subscriber("tracking/human_vel2d",
                                              Vector3, self.human_track_vel_cb)
        self.track_status_sub = rospy.Subscriber("tracking/status",
                                                 String, self.human_track_status_cb)
        self.odom_sub = rospy.Subscriber("odom",
                                         Odometry, self.odom_cb)
        # subscribers to human input
        self.human_input_ort_sub = rospy.Subscriber("human_input/tilt",
                                                    Vector3, self.human_input_tilt_cb)
        self.human_input_gesture_sub = rospy.Subscriber("human_input/gesture",
                                                        Int8, self.human_input_gesture_cb)
        self.human_input_mode_sub = rospy.Subscriber("human_input/button",
                                                     Bool, self.human_input_mode_cb)

        # subscriber to state control
        self.state_control_sub = rospy.Subscriber("state_control/set_state",
                                                  Int8, self.set_state_cb)
        self.cmd_state_control_sub = rospy.Subscriber("cmd_state_control/set_state",
                                                      Int8, self.set_cmd_state_cb)
        # subscriber to haptic tether control
        self.follower_mode_control_sub = rospy.Subscriber("state_control/set_follower_mode",
                                                          Int8, self.follower_mode_control_cb)

        # subscribe to the bumper event
        self.bumper_event_sub = rospy.Subscriber("mobile_base/events/bumper",
                                                 BumperEvent, self.bumper_event_cb)

        # publisher to robot velocity
        self.robot_state_pub = rospy.Publisher("robot_follower_state",
                                               Int8, queue_size=1)
        self.robot_vel_pub = rospy.Publisher("cmd_vel",
                                             Twist, queue_size=1)
        self.haptic_msg_pub = rospy.Publisher("haptic_control",
                                              haptic_msg, queue_size=1)

        # publisher to system message
        self.sys_msg_pub = rospy.Publisher("sys_message",
                                           String, queue_size=1)

        # publisher to base LED and digital output
        self.LED1_pub = rospy.Publisher("mobile_base/commands/led1",
                                        Led, queue_size=1)
        self.digital_out_pub = rospy.Publisher("mobile_base/commands/digital_output",
                                               DigitalOutput, queue_size=1)

    # call back functions
    def follower_mode_control_cb(self, follower_mode_control_msg):
        self.set_follower_mode = follower_mode_control_msg.data

    def human_track_pose_cb(self, msg):
        self.human_pose = msg

    def human_track_vel_cb(self, vel_msg):
        self.human_vel = vel_msg

    def human_track_status_cb(self, msg):
        self.track_status = msg.data

    def human_input_tilt_cb(self, msg):
        self.human_input_tilt = msg
        self.human_input_tilt.x += self.roll_center_offset

    def human_input_gesture_cb(self, msg):
        self.human_input_gesture = msg.data

        # check if set to start/stop
        if gesture_dict[self.human_input_gesture] == "twist":
            if self.state == "Idle":
                self.set_state = 1
            else:
                self.set_state = 0

    def human_input_mode_cb(self, msg):
        if msg.data:
            self.flag_button_pressed = True
        else:
            self.flag_button_pressed = False

    def set_state_cb(self, msg):
        self.set_state = msg.data
        rospy.loginfo("set state to be %s", states_all[self.set_state])

    def set_cmd_state_cb(self, msg):
        self.set_cmd_state = msg.data
        self.cmd_state = cmd_states_all[self.set_cmd_state]
        rospy.loginfo("set command state to %s", self.cmd_state)

    def odom_cb(self, msg):
        self.measured_vel = msg.twist.twist

    def bumper_event_cb(self, bumper_event_msg):
        self.bumper_event = bumper_event_msg

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

    def send_haptic_msg(self, rep, t_render, t_pause):
        new_msg = haptic_msg()

        # calculate robot position in human's frame of reference
        x_r = -self.human_pose.x * np.cos(self.human_pose.theta) - self.human_pose.y * np.sin(self.human_pose.theta)
        y_r = self.human_pose.x * np.sin(self.human_pose.theta) - self.human_pose.y * np.cos(self.human_pose.theta)

        phi = np.arctan2(y_r, x_r)

        # calcualte direction and amplitude based on human position
        id_map = np.array([1, 5, 2, 4, 0, 6, 3, 7])
        for id in range(0, 8):
            phi_c = id * np.pi / 4.0
            if np.abs(phi - phi_c) <= np.pi / 8.0:
                new_msg.direction = id_map[id]
                break

        new_msg.amplitude = 1.5  # self.human_pose.y / self.haptic_amp_thresh

        # send haptic control message
        new_msg.repetition = rep
        new_msg.period_render = t_render
        new_msg.period_pause = t_pause

        self.haptic_msg_pub.publish(new_msg)

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
            self.set_state = -1
            # check if already have human in vision
            if self.track_status == "Lost":
                rospy.logwarn("Cannot start following! Human not found!")
                self.sys_msg_pub.publish("Cannot start following! Human not found!")
            else:
                # set to state follow
                self.state = "Follow"
        elif self.set_state == 4:
            # directly to go state teleop
            self.state = "Teleop"

    def follow(self):
        # check if need to switch state
        if self.track_status == "Lost":
            self.lost_vision_count += 1
            if self.lost_vision_count >= self.lost_vision_count_limit:
                if self.set_follower_mode == 2:
                    # send haptic signal
                    self.send_haptic_msg(3, 1.0, 1.0)

                # set robot to stop and go to state lost vision
                self.lost_vision_count = 0
                self.send_vel_cmd(0, 0, 0.5)
                self.state = "LostVision"
                rospy.logwarn("Lost vision of human!")
                self.sys_msg_pub.publish("Lost vision of human!")
                return
        else:
            self.lost_vision_count = 0

        # check if human is walking too fast
        if np.abs(self.cmd_vel.linear.x) > self.turtlebot_vel_max:
            self.too_fast_count += 1
            if self.too_fast_count >= self.too_fast_count_limit:
                if self.set_follower_mode == 2:
                    # send haptic signal
                    self.send_haptic_msg(2, 0.5, 0.5)

                self.too_fast_count = 0
                self.sys_msg_pub.publish("Human walking too fast!")
        else:
            self.too_fast_count = 0

        # check if get stuck
        if self.bumper_event.state == BumperEvent.PRESSED:
            self.stuck_count += 1
            if self.stuck_count >= self.stuck_count_limit:
                if self.set_follower_mode == 2:
                    # send haptic signal
                    self.send_haptic_msg(3, 1.0, 1.0)

                self.stuck_backup_count = 0
                self.send_vel_cmd(-0.5, 0, 1.0)
                self.state = "GetStuck"
                # rospy.logwarn("Robot stuck!")
                self.sys_msg_pub.publish("Robot stuck!")
                return
        else:
            self.stuck_count = 0

        self.check_set_state()

        # calculate desired velocity to follow
        vx = -self.kp_linear * (self.dist_desired - self.human_pose.y) + self.kd_linear * self.human_vel.y
        omg = -self.kp_angular * self.human_pose.x - self.kd_angular * self.human_vel.x

        # add in some damping to the system
        vx -= self.kb_linear * self.measured_vel.linear.x
        omg -= self.kb_angular * self.measured_vel.angular.z

        self.send_vel_cmd(vx, omg)

    def lost_vision(self):
        # check if regains sight, and in proper range
        if self.track_status == "Find":
            if self.dist_range_min < self.human_pose.y < self.dist_range_max:
                # go back to following
                self.state = "Follow"

        # do teleop
        if self.set_follower_mode == 2:
            self.teleop()
        else:
            self.check_set_state()

    def get_stuck(self):
        # backup and try to follow again
        if self.stuck_backup_count < self.stuck_backup_count_limit:
            self.stuck_backup_count += 1
        else:
            # check if human is within distance
            if self.track_status == "Find" and self.human_pose.y <= self.dist_stuck_resume_max:
                rospy.logwarn('Prepare to switch to follow')
                self.sys_msg_pub.publish("Continue to follow")

                self.state = "Follow"

            # also can be set to follow manually
            if self.set_state == 1:
                if self.track_status == "Find":
                    rospy.logwarn('Prepare to switch to follow')
                    self.sys_msg_pub.publish("Prepare to switch to follow")
                    if self.dist_range_min < self.human_pose.y < self.dist_range_max:
                        self.set_state = -1
                        self.state = "Follow"

        # do teleoperation
        if self.set_follower_mode == 2:
            self.teleop()
        else:
            self.check_set_state()

    # method for digital output
    def digital_write(self, channel, value):
        t_digital_out = DigitalOutput()
        t_digital_out.mask[channel] = 1
        t_digital_out.values[channel] = value

        self.digital_out_pub.publish(t_digital_out)

    def teleop(self):
        # rospy.loginfo("in teleoperation")
        if self.set_follower_mode != 0:
            # do not switch to other states when in teleoperation condition
            self.check_set_state()

        if self.flag_button_pressed:
            if self.flag_latch_cmd_vel:
                if self.human_input_tilt.x > self.roll_deadband and self.tele_vel.linear.x < self.turtlebot_vel_max:
                    self.tele_vel.linear.x += self.tele_vel_inc_linear
                elif self.human_input_tilt.x < -self.roll_deadband and self.tele_vel.linear.x > -self.turtlebot_vel_max:
                    self.tele_vel.linear.x -= self.tele_vel_inc_linear
                vx = self.tele_vel.linear.x
            else:
                if self.human_input_tilt.x > self.roll_deadband:
                    vx = self.roll_to_linear_scale * (self.human_input_tilt.x - self.roll_offset)
                elif self.human_input_tilt.x < -self.roll_deadband:
                    vx = self.roll_to_linear_scale * (self.human_input_tilt.x + self.roll_offset)
                else:
                    vx = 0

            if self.human_input_tilt.y > self.pitch_deadband:
                self.tele_vel.angular.z = self.pitch_to_angular_scale * (self.human_input_tilt.y - self.pitch_offset)
            elif self.human_input_tilt.y < -self.pitch_deadband:
                self.tele_vel.angular.z = self.pitch_to_angular_scale * (self.human_input_tilt.y + self.pitch_offset)
            else:
                self.tele_vel.angular.z = 0

            self.send_vel_cmd(vx, self.tele_vel.angular.z)
        else:
            self.send_vel_cmd(0, 0)

    def update(self):

        # run over states
        current_state = Int8()
        if self.state == "Idle":
            self.idle()
            current_state.data = 0
            self.LED1_pub.publish(Led.BLACK)
            self.digital_write(0, 1)
        elif self.state == "Follow":
            self.follow()
            current_state.data = 1
            self.LED1_pub.publish(Led.GREEN)
            self.digital_write(0, 1)
        elif self.state == "LostVision":
            self.lost_vision()
            current_state.data = 2
            self.LED1_pub.publish(Led.ORANGE)
            self.digital_write(0, 1)
        elif self.state == "GetStuck":
            self.get_stuck()
            current_state.data = 3
            self.LED1_pub.publish(Led.RED)
            self.digital_write(0, 0)
        elif self.state == "Teleop":
            self.teleop()
            current_state.data = 4
            self.LED1_pub.publish(Led.BLACK)
        else:
            rospy.logerr("Unknown state!")

        # publish the state
        self.robot_state_pub.publish(current_state)

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
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        follower.update()
        rate.sleep()
