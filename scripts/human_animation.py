#!/usr/bin/env python

import rospy
import numpy
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
# from gazebo_msgs.msg import ModelStates


def fetch_param(name, default):
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        print "parameter [%s] not defined, using default" % name
    return default


if __name__ == "__main__":
    rospy.init_node("simple_human_animation")
    # state_sub = rospy.Subscriber("gazebo/set_model_state", ModelStates, human_track_cb)
    traj_pub = rospy.Publisher("gazebo/set_model_state", ModelState, queue_size=1)
    set_state_client = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    # fetch initial state or use default
    init_state_default = ModelState()
    init_state_default.pose.position = (0, 1.53678, 0)
    init_state_default.pose.orientation = (0, 0, 0, 1)
    init_state_default.twist.linear = (0, 0, 0)
    init_state_default.twist.angular = (0, 0, 0)

    init_state = fetch_param("~init_state_human", init_state_default)
    R = fetch_param("~traj_radius", 5)
    omega = fetch_param("~traj_rate", 0.1)

    # move along a circle with radius R
    center_x = init_state.pose.position[0] + R
    center_y = init_state.pose.position[1]

    rospy.sleep(1.0)
    rate = rospy.Rate(20)
    t_init = rospy.get_time()
    
    while not rospy.is_shutdown():
        t_now = rospy.get_time() - t_init
        theta = omega * t_now

        # calculate new pose
        new_state = ModelState()
        new_state.model_name = "simple_human"
        new_state.reference_frame = "world"

        new_pos = Pose()
        new_pos.position.x = center_x - R*numpy.cos(theta)
        new_pos.position.y = center_y + center_y + R * numpy.sin(theta)
        new_pos.orientation.z = -numpy.sin(theta/2.0)
        new_pos.orientation.w = numpy.cos(theta/2.0)

        new_state.pose = new_pos

        # print new_state

        # publish the new pose
        # traj_pub.publish(new_state)
        set_state_client.call(new_state)
        rate.sleep()
