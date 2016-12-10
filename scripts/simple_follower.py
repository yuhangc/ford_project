#!/usr/bin/env python

import rospy
import numpy as np

from blinkstick import blinkstick

from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent

from ford_project.msg import haptic_msg

states_all = {0: "Idle",
              1: "Follow",
              2: "LostVision",
              3: "StuckObstacle",
              4: "StuckSoftware",
              5: "StuckTooFast",
              6: "Teleop"}  # teleop state is only for testing

cmd_states_all = {0: "Idle",
                  1: "SendOnce",
                  2: "sendPeriod"}

input_mode_all = {1: "gesture_control",
                  0: "tilt_control"}


class SimpleFollower:
    def __init__(self):
        # initialize the blinkstick
        self.vision_led = blinkstick.find_first()

        # set initial states
        self.state = rospy.get_param("~state_init", "Idle")
        self.state_last = ""
        self.cmd_state = rospy.get_param("~cmd_state_init", "Idle")

        # flag that indicates whether haptic tether is enables
        # 0 - Teleoperation
        # 1 - Autonomous
        # 2 - Haptic Tether
        self.follower_mode = 2

        # timer for state machine
        self.cmd_state_timer = rospy.get_time()
        self.cmd_state_period = 0.0

        # time in following mode
        self.dt_following = 0.0

        # loop counters
        self.stuck_count = 0
        self.stuck_backup_count = 0
        self.lost_vision_count = 0
        self.too_fast_count = 0
        self.slow_down_count = 0
        self.too_fast_pause_count = 0

        self.flag_too_fast_check_pause = False

        self.stuck_backup_count_limit = 50
        self.stuck_count_limit = rospy.get_param("~stuck_count_limit", 25)
        self.lost_vision_count_limit = rospy.get_param("~lost_vision_count_limit", 25)
        self.too_fast_count_limit_low = rospy.get_param("~too_fast_count_limit_low", 25)
        self.too_fast_count_limit_high = rospy.get_param("~too_fast_count_limit_high", 45)
        self.too_fast_pause_count_limit = rospy.get_param("~too_fast_pause_count_limit", 65)
        self.slow_down_count_limit = rospy.get_param("~slow_down_count_limit", 25)
        self.turtlebot_vel_max = rospy.get_param("~turtlebot_vel_max", 0.7)

        # a little bit of hysteresis
        self.too_fast_threshold_low = rospy.get_param("~too_fast_threshold_low", self.turtlebot_vel_max * 1.4)
        self.too_fast_threshold_high = rospy.get_param("~too_fast_threshold_high", self.turtlebot_vel_max * 1.5)

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
        self.dist_range_max = rospy.get_param("~dist_range_max", 1.9)

        self.dist_stuck_resume_max = rospy.get_param("~dist_stuck_resume_max", 1.0)

        # variables for tilt control
        self.roll_to_linear_scale = rospy.get_param("~roll_to_linear_scale", -  1.0)
        self.pitch_to_angular_scale = rospy.get_param("~pitch_to_angular_scale", -2.0)
        self.pitch_deadband = rospy.get_param("~pitch_deadband", 0.2)
        self.roll_deadband = rospy.get_param("~roll_deadband", 0.2)
        self.pitch_offset = rospy.get_param("~pitch_offset", 0.15)
        self.roll_offset = rospy.get_param("~roll_offset", 0.1)
        self.roll_center_offset = rospy.get_param("~roll_center_offset", -0.05)
        self.pitch_center_offset = rospy.get_param("~pitch_center_offset", -0.1)
        self.flag_reverse_mapping = rospy.get_param("~reverse_mapping", False)

        # randomization parameter for "get stuck"
        # stuck mode - 0: no software stuck, 1 - too fast stuck, 2 - random stuck, 3 - both
        self.stuck_mode = rospy.get_param("~stuck_mode", 0)
        self.num_stuck_total = rospy.get_param("~number_get_stuck_total", 5)
        self.period_stuck_min = rospy.get_param("~period_stuck_min", 20)
        self.period_stuck_max = rospy.get_param("~period_stuck_max", 40)

        self.flag_soft_stuck = False
        self.num_stuck = 0
        self.t_following_start = 0.0
        self.period_stuck = 0.0

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
        self.button_event_sub = rospy.Subscriber("human_input/button_event",
                                                 Int8, self.button_event_cb)

        # subscriber to reverse mapping
        self.reverse_mapping_sub = rospy.Subscriber("human_input/reverse_mapping",
                                                    Bool, self.reverse_mapping_cb)

        # subscriber to state control
        self.state_control_sub = rospy.Subscriber("state_control/set_state",
                                                  Int8, self.set_state_cb)
        self.cmd_state_control_sub = rospy.Subscriber("cmd_state_control/set_state",
                                                      Int8, self.set_cmd_state_cb)
        # subscriber to haptic tether control
        self.follower_mode_control_sub = rospy.Subscriber("state_control/set_follower_mode",
                                                          Int8, self.follower_mode_control_cb)

        # subscriber to "stuck mode" control
        self.stuck_mode_control_sub = rospy.Subscriber("state_control/set_stuck_mode",
                                                       Int8, self.stuck_mode_control_cb)

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

    # call back functions
    def follower_mode_control_cb(self, follower_mode_control_msg):            
        self.follower_mode = follower_mode_control_msg.data

    def stuck_mode_control_cb(self, stuck_mode_control_msg):
        self.stuck_mode = stuck_mode_control_msg.data
        self.sys_msg_pub.publish("switch to stuck mode " + str(self.stuck_mode))

    def human_track_pose_cb(self, msg):
        self.human_pose = msg

    def human_track_vel_cb(self, vel_msg):
        self.human_vel = vel_msg

    def human_track_status_cb(self, msg):
        # # change LED color when status change
        # if msg.data != self.track_status:
        #     if msg.data == "Find":
        #         self.vision_led.set_color(name="green")
        #     else:
        #         self.vision_led.set_color(name="red")

        self.track_status = msg.data

    def human_input_tilt_cb(self, msg):
        self.human_input_tilt = msg
        self.human_input_tilt.x += self.roll_center_offset
        self.human_input_tilt.y += self.pitch_center_offset

    def human_input_gesture_cb(self, msg):
        self.human_input_gesture = msg.data

    def button_event_cb(self, msg):
        if msg.data == 2:
            if self.state == "Follow":
                self.set_state = 0
            else:
                self.set_state = 1

    def human_input_mode_cb(self, msg):
        if msg.data:
            self.flag_button_pressed = True
        else:
            self.flag_button_pressed = False

    def reverse_mapping_cb(self, msg):
        if msg.data:
            self.flag_reverse_mapping = True
        else:
            self.flag_reverse_mapping = False

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

    def send_haptic_msg(self, dir, rep, t_render, t_pause):
        new_msg = haptic_msg()

        # calculate robot position in human's frame of reference
        x_r = -self.human_pose.x * np.cos(-self.human_pose.theta) - self.human_pose.y * np.sin(-self.human_pose.theta)
        y_r = self.human_pose.x * np.sin(-self.human_pose.theta) - self.human_pose.y * np.cos(-self.human_pose.theta)

        phi = np.arctan2(y_r, x_r)
        if phi < 0:
            phi += 2 * np.pi

        self.sys_msg_pub.publish("(" + str(x_r) + ", " + str(y_r) + ")    " + str(phi))
        rospy.logwarn("(%f, %f),   %f", x_r, y_r, phi)

        # if specified direction and render time
        if dir != -1:
            new_msg.direction = dir
            new_msg.amplitude = 2.5
            new_msg.repetition = rep
            new_msg.period_render = t_render
            new_msg.period_pause = t_pause

            self.haptic_msg_pub.publish(new_msg)
            return

        # otherwise calcualte direction and amplitude based on human position
        # id_map = np.array([1, 5, 2, 4, 0, 6, 3, 7])
        if 0.5 * np.pi < phi < 1.5 * np.pi:
            new_msg.direction = 2
        else:
            new_msg.direction = 3

        new_msg.amplitude = 2.5  # self.human_pose.y / self.haptic_amp_thresh

        # make the render time proportional to the magnitude of angle
        delta_phi = np.abs(phi - 0.5 * np.pi)
        if delta_phi > np.pi:
            delta_phi = np.pi * 2.0 - delta_phi

        new_msg.period_render = delta_phi / np.pi * 1.5

        # send haptic control message
        new_msg.repetition = rep
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

    def set_stuck_param(self):
        if (self.stuck_mode & 0x02) == 0:
            return

        self.t_following_start = rospy.get_time()
        self.period_stuck = np.random.randint(self.period_stuck_min, self.period_stuck_max)
        self.sys_msg_pub.publish("start time: " + str(self.t_following_start) + "period: " + str(self.period_stuck))

    def set_to_follow(self):
        self.state = "Follow"
        self.set_stuck_param()

        # pause too fast check for 0.5s
        self.flag_too_fast_check_pause = True
        self.too_fast_pause_count = 0

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
                self.set_to_follow()
        elif self.set_state == 4:
            # directly to go state teleop
            self.state = "Teleop"

    def follow(self):
        # update time following
        self.dt_following = rospy.get_time() - self.t_following_start
        # check if need to switch state
        if self.track_status == "Lost":
            self.lost_vision_count += 1
            if self.lost_vision_count >= self.lost_vision_count_limit:
                if self.follower_mode == 2:
                    # send haptic signal
                    self.send_haptic_msg(-1, 3, -1, 1.0)

                # update stuck timer
                self.period_stuck -= self.dt_following

                # set robot to stop and go to state lost vision
                self.lost_vision_count = 0
                self.send_vel_cmd(0, 0)
                self.state = "LostVision"
                rospy.logwarn("Lost vision of human!")
                self.sys_msg_pub.publish("Lost vision of human!")
                return
        else:
            self.lost_vision_count = 0

        # check if pause too fast check
        if self.flag_too_fast_check_pause:
            self.too_fast_pause_count += 1
            if self.too_fast_pause_count >= self.too_fast_pause_count_limit:
                self.flag_too_fast_check_pause = False

        # check if human is walking too fast
        if not self.flag_too_fast_check_pause and np.abs(self.cmd_vel.linear.x) > self.too_fast_threshold_high:
            self.too_fast_count += 1
            if self.too_fast_count == self.too_fast_count_limit_low:
                if (self.stuck_mode & 0x01) != 0 and self.follower_mode == 2:
                    # send haptic signal
                    self.send_haptic_msg(1, 2, 0.5, 0.5)

                self.sys_msg_pub.publish("Human walking too fast!")

            if (self.stuck_mode & 0x01) != 0 and self.too_fast_count >= self.too_fast_count_limit_high:
                # force the robot to "stuck" since human is walking too fast
                self.too_fast_count = 0
                self.send_vel_cmd(0.0, 0.0)

                self.state = "StuckTooFast"
                self.flag_soft_stuck = True
                self.sys_msg_pub.publish("Robot stuck (human too fast)!")
                return
        elif np.abs(self.cmd_vel.linear.x) <= self.too_fast_threshold_low:
            self.slow_down_count += 1
            if self.slow_down_count >= self.slow_down_count_limit:
                self.too_fast_count = 0
                self.slow_down_count = 0

        # check for "stuck" set by software
        if (self.stuck_mode & 0x02) != 0:
            # use software to simulate stuck
            if self.dt_following > self.period_stuck:
                self.num_stuck += 1

                # send haptic signal
                if self.follower_mode == 2:
                    self.send_haptic_msg(-1, 3, 1.0, 1.0)

                self.send_vel_cmd(0.0, 0.0)

                self.state = "StuckSoftware"
                self.flag_soft_stuck = True
                self.sys_msg_pub.publish("Robot stuck (software)!")
                return

        # check for real stuck
        if self.bumper_event.state == BumperEvent.PRESSED:
            self.stuck_count += 1
            if self.stuck_count >= self.stuck_count_limit:
                if self.follower_mode == 2:
                    # send haptic signal
                    self.send_haptic_msg(-1, 3, 1.0, 1.0)

                self.stuck_backup_count = 0
                self.send_vel_cmd(-0.5, 0, 1.0)
                self.state = "StuckObstacle"
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
                self.t_following_start = rospy.get_time()

        # do teleop
        if self.follower_mode == 2:
            self.teleop()
        else:
            self.check_set_state()

    def stuck(self):
        # backup and try to follow again
        if not self.flag_soft_stuck and self.stuck_backup_count < self.stuck_backup_count_limit:
            self.stuck_backup_count += 1
        else:
            # has to be set to follow manually
            if self.set_state == 1:
                if self.track_status == "Find":
                    self.sys_msg_pub.publish("Prepare to switch to follow")
                    if self.dist_range_min < self.human_pose.y < self.dist_range_max:
                        self.set_state = -1
                        self.set_to_follow()
                    else:
                        # clear the set state message
                        self.set_state = -1

        # do teleoperation
        if self.follower_mode == 2:
            self.teleop()
        else:
            self.check_set_state()

    def teleop(self):
        # rospy.loginfo("in teleoperation")
        if self.follower_mode != 0:
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

            # reverse mapping if necessary
            if self.flag_reverse_mapping:
                vx = -vx
                self.tele_vel.angular.z = -self.tele_vel.angular.z

            self.send_vel_cmd(vx, self.tele_vel.angular.z)
        else:
            self.send_vel_cmd(0, 0)

    def update(self):

        # run over states
        current_state = Int8()
        if self.state == "Idle":
            self.idle()
            current_state.data = 0
        elif self.state == "Follow":
            self.follow()
            current_state.data = 1
        elif self.state == "LostVision":
            self.lost_vision()
            current_state.data = 2
        elif self.state == "StuckObstacle":
            self.stuck()
            current_state.data = 3
        elif self.state == "StuckSoftware":
            self.stuck()
            current_state.data = 4
        elif self.state == "StuckTooFast":
            self.stuck()
            current_state.data = 5
        elif self.state == "Teleop":
            self.teleop()
            current_state.data = 6
        else:
            rospy.logerr("Unknown state!")

        # publish the state
        self.robot_state_pub.publish(current_state)

        # change LED color based on state
        if self.state != self.state_last:
            if self.state == "Idle":
                self.vision_led.set_color(name="yellow")
            elif self.state == "Follow":
                self.vision_led.set_color(name="green")
            elif self.state == "LostVision":
                self.vision_led.set_color(name="red")
            elif self.state == "StuckObstacle":
                self.vision_led.set_color(name="purple")
            elif self.state == "StuckSoftware":
                self.vision_led.set_color(name="purple")
            elif self.state == "StuckTooFast":
                self.vision_led.set_color(name="purple")
            elif self.state == "Teleop":
                self.vision_led.set_color(name="blue")

        self.state_last = self.state

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

    # turn off the LED
    follower.vision_led.turn_off()
