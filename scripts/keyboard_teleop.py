#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import sys, select, termios, tty

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState

msg = """
Control the virtual human
---------------------------
Moving around:
u i o
j k l
m , .
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly
CTRL-C to quit
"""
moveBindings = {
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    ',': (-1, 0),
    '.': (-1, 1),
    'm': (-1, -1),
}
speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

# set model state client
set_state_client = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
get_state_client = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    return key


speed = .2
turn = 1


def vels(speed, turn):
    return "currently: speed %s   turn %s" % (speed, turn)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('turtlebot_teleop')
    pub = rospy.Publisher('/human_movable/cmd_vel', Twist, queue_size=5)
    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    try:
        print msg
        print vels(speed, turn)
        while 1:
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                count = 0
                print vels(speed, turn)
                if status == 14:
                    print msg
                status = (status + 1) % 15
            elif key == 'k':
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
            elif key == ' ':
                # move human up stairs
                old_state = get_state_client.call("human_movable", "world")

                new_state = ModelState()
                new_state.model_name = "human_movable"
                new_state.reference_frame = "world"

                new_state.pose = old_state.pose
                new_state.twist = old_state.twist
                new_state.pose.position.x -= 0.5
                new_state.pose.position.z += 0.15

                set_state_client.call(new_state)
            else:
                count += 1
                if count > 4:
                    x = 0
                    th = 0
                if key == '\x03':
                    break

            target_speed = speed * x
            target_turn = turn * th

            if target_speed > control_speed:
                control_speed = min(target_speed, control_speed + 0.2)
            elif target_speed < control_speed:
                control_speed = max(target_speed, control_speed - 0.2)
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min(target_turn, control_turn + 0.5)
            elif target_turn < control_turn:
                control_turn = max(target_turn, control_turn - 0.5)
            else:
                control_turn = target_turn

            twist = Twist()
            twist.linear.x = control_speed
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = control_turn
            pub.publish(twist)
            # print("loop: {0}".format(count))
            # print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
            # print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))

    except:
        # print e
        pass

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
