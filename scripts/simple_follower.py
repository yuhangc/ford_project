#!/usr/bin/env python

import rospy
from smach import State, StateMachine
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from ford_project.msg import haptic_msg
from std_msgs.msg import Char

class Idle(State):
    def __init__(self):
        State.__init__(self, outcomes=["start_following"])

    def execute(self, ud):
        # do nothing
        return

class Following(State):
    def __init__(self):
        State.__init__(self, outcomes=["following_failed", "stop_following"])

    def execute(self, ud):
        # do nothing
        return

class Send_FB(State):
    def __init__(self):
        State.__init__(self, outcomes=["send_success", "send_failed"])

    def execute(self, ud):
        # do nothing
        return

class Wait_CMD(State):
    def __init__(self):
        State.__init__(self, outcomes=["wait_too_long", "cmd_received"])

    def execute(self, ud):
        # do nothing
        return

class Respond(State):
    def __init__(self):
        State.__init__(self, outcomes=["success", "failed"])

    def execute(self, ud):
        # do nothing
        return


if __name__ == "__main__":
    # init the node
    rospy.init_node("simple_follower")

    # define subscribers and publishers

    sm = StateMachine()

    with sm:
        StateMachine.add("IDLE", Idle(),
                         transitions={ "start_following": "FOLLOWING" })
        StateMachine.add("FOLLOWING", Following(),
                         transitions={ "following_failed": "SEND_FB",
                                       "stop_following": "IDLE" })
        StateMachine.add("SEND_FB", Send_FB(),
                         transitions={ "send_success": "WAIT_CMD",
                                       "send_failed": "SEND_FB" })
        StateMachine.add("WAIT_CMD", Wait_CMD(),
                         transitions={ "wait_too_long": "RESPOND",
                                       "cmd_received": "RESPOND" })
        StateMachine.add("RESPOND", Respond(),
                         transitions={ "success": "FOLLOWING",
                                       "failed": "SEND_FB" })

    sm.execute()
