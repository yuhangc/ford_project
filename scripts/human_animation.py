#!/usr/bin/env python

import rospy
import numpy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Pose2D
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


def fetch_param(name, default):
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        print "parameter [%s] not defined, using default" % name
    return default


class HumanAnimation:
    def __init__(self, file_name):
        # initialize lists
        self.way_points = []
        self.type = []
        self.duration = []

        # open file
        f = open("../human_traj/" + file_name, "r+")

        # read in and parse the trajectory file
        self.num_points = 0
        for line in f:
            pos_data = line.split(',')

            pose_new = Pose2D()
            pose_new.x = float(pos_data[0])
            pose_new.y = float(pos_data[1])
            pose_new.theta = float(pos_data[2])
            self.way_points.append(pose_new)

            if self.num_points > 0:
                # also read in segment type and duration
                self.type.append(pos_data[3][1])
                self.duration.append(float(pos_data[4]))

            self.num_points += 1
        self.num_points -= 1

        # set model state client
        self.set_state_client = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        # initialize variables for update method
        self.t_start = 0.0
        self.id = 0

    def update(self):
        # initialize t_start
        if self.t_start == 0:
            self.t_start = rospy.get_time()

        # return if run out of segments
        if self.id >= self.num_points:
            return

        # check if enters next segment
        t_now = rospy.get_time() - self.t_start
        if t_now > self.duration[self.id]:
            self.t_start += self.duration[self.id]
            t_now -= self.duration[self.id]
            self.id += 1

        # return if run out of segments
        if self.id >= self.num_points:
            return

        # interpolate to get current pose
        pose_desired = Pose2D()
        pos1 = self.way_points[self.id]
        pos2 = self.way_points[self.id+1]
        if self.type[self.id] == 'L':
            pose_desired.x = pos1.x + t_now/self.duration[self.id]*(pos2.x-pos1.x)
            pose_desired.y = pos1.y + t_now/self.duration[self.id]*(pos2.y-pos1.y)
            pose_desired.theta = pos1.theta + t_now/self.duration[self.id]*(pos2.theta-pos1.theta)
        else:
            r = (pos1.x - pos2.x)/(numpy.cos(pos2.theta) - numpy.cos(pos1.theta))
            pose_desired.theta = pos1.theta + t_now/self.duration[self.id]*(pos2.theta-pos1.theta)
            pose_desired.x = pos1.x + r * (numpy.cos(pos1.theta) - numpy.cos(pose_desired.theta))
            pose_desired.y = pos1.y + r * (numpy.sin(pos1.theta) - numpy.sin(pose_desired.theta))

        # set and publish new state
        new_pose = Pose()
        new_pose.position.x = pose_desired.x
        new_pose.position.y = pose_desired.y
        new_pose.orientation.z = numpy.sin(pose_desired.theta / 2.0)
        new_pose.orientation.w = numpy.cos(pose_desired.theta / 2.0)

        new_state = ModelState()
        new_state.model_name = "simple_human"
        new_state.reference_frame = "world"
        new_state.pose = new_pose

        print t_now, self.id, self.num_points, pos1.theta, pose_desired.theta
        print new_state.pose

        self.set_state_client.call(new_state)


if __name__ == "__main__":
    rospy.init_node("simple_human_animation")

    animation = HumanAnimation("traj1")

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        animation.update()
        rate.sleep()
