#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from gazebo_msgs.msg import ModelStates


class imuTeleop:
    def __init__(self):
        # desired velocity for publish
        self.cmd_vel = Twist()

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

        # data saving object
        data_file_path = rospy.get_param("~data_file_path", "")
        self.data_saver = open(data_file_path + "/robot_gazebo.txt", 'w')
        self.data_save_counter = 0

        # human input variables
        self.human_input_tilt = Vector3()
        self.flag_button_pressed = False

        # subscribers to human input
        self.human_input_ort_sub = rospy.Subscriber("human_input/tilt",
                                                    Vector3, self.human_input_tilt_cb)
        self.human_input_mode_sub = rospy.Subscriber("human_input/button",
                                                     Bool, self.human_input_mode_cb)

        # subscriber to reverse mapping
        self.reverse_mapping_sub = rospy.Subscriber("human_input/reverse_mapping",
                                                    Bool, self.reverse_mapping_cb)

        # subscriber to robot state
        self.robot_state_sub = rospy.Subscriber("/gazebo/model_states",
                                                ModelStates, self.robot_state_cb)

        # publisher to robot velocity
        self.robot_vel_pub = rospy.Publisher("cmd_vel",
                                             Twist, queue_size=1)

    # call back functions
    def human_input_tilt_cb(self, msg):
        self.human_input_tilt = msg
        self.human_input_tilt.x += self.roll_center_offset
        self.human_input_tilt.y += self.pitch_center_offset

    def reverse_mapping_cb(self, msg):
        if msg.data:
            self.flag_reverse_mapping = True
        else:
            self.flag_reverse_mapping = False

    def human_input_mode_cb(self, msg):
        if msg.data:
            self.flag_button_pressed = True
        else:
            self.flag_button_pressed = False

    def robot_state_cb(self, msg):
        self.data_save_counter += 1
        if self.data_save_counter != 10:
            return

        self.data_save_counter = 0
        for i in xrange(len(msg.name)):
            if msg.name[i] == "mobile_base":
                # save pose to file
                out_str = "{:03f} {:03f} {:03f}\n".format(rospy.get_time(), msg.pose[i].position.x, msg.pose[i].position.y)
                self.data_saver.writelines(out_str)
                break

    # utility functions
    def send_vel_cmd(self, vx, omg):
        # set and send desired velocity
        self.cmd_vel.linear.x = vx
        self.cmd_vel.angular.z = omg

        self.robot_vel_pub.publish(self.cmd_vel)

    def teleop(self):
        if self.flag_button_pressed:
            if self.human_input_tilt.x > self.roll_deadband:
                vx = self.roll_to_linear_scale * (self.human_input_tilt.x - self.roll_offset)
            elif self.human_input_tilt.x < -self.roll_deadband:
                vx = self.roll_to_linear_scale * (self.human_input_tilt.x + self.roll_offset)
            else:
                vx = 0

            if self.human_input_tilt.y > self.pitch_deadband:
                vz = self.pitch_to_angular_scale * (self.human_input_tilt.y - self.pitch_offset)
            elif self.human_input_tilt.y < -self.pitch_deadband:
                vz = self.pitch_to_angular_scale * (self.human_input_tilt.y + self.pitch_offset)
            else:
                vz = 0

            # reverse mapping if necessary
            if self.flag_reverse_mapping:
                vx = -vx
                vz = -vz

            self.send_vel_cmd(vx, vz)
        else:
            self.send_vel_cmd(0, 0)

    def stop_data_saving(self):
        self.data_saver.close()

if __name__ == "__main__":
    # initialize node
    rospy.init_node("simple_follower")

    # create a simple follower object
    teleop = imuTeleop()

    # loop rate 100 Hz
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        teleop.teleop()
        rate.sleep()

    teleop.stop_data_saving()
