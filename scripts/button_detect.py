#!/usr/bin/env python

import rospy

from std_msgs.msg import Bool
from std_msgs.msg import Int8


class ButtonDetector:
    def __init__(self):
        self.state = 0
        self.state_next = 0
        self.flag_button_pressed = False

        # timers
        self.t_button_press = 0.0
        self.t_button_release = 0.0
        self.t_state = 0.0

        # click counter
        self.click_count = 0

        # parameters
        self.dt_debounce = rospy.get_param("~dt_debounce", 0.05)
        self.dt_double_click_max = rospy.get_param("~dt_double_click_max", 0.6)
        self.dt_single_click_max = rospy.get_param("~dt_single_click_max", 0.8)

        # subscriber and publisher
        self.human_input_mode_sub = rospy.Subscriber("human_input/button",
                                                     Bool, self.human_input_mode_cb)

        self.button_event_publisher = rospy.Publisher("human_input/button_event",
                                                      Int8, queue_size=1)

    def human_input_mode_cb(self, msg):
        if msg.data:
            self.flag_button_pressed = True
        else:
            self.flag_button_pressed = False

        self.update()

    def update(self):
        # state 0 - button not pressed
        # state 1 - button clicked once, wait to debounce
        # state 2 - button clicked and hold down, wait to release
        # state 3 - button clicked once and released, wait for second click
        if self.state == 0:
            # check for button press
            if self.flag_button_pressed:
                # publish the single click event
                self.state = 1
                self.state_next = 2
                self.t_button_press = rospy.get_time()
        elif self.state == 1:
            # check for debounce timer
            self.t_state = rospy.get_time() - self.t_button_press
            if self.t_state >= self.dt_debounce:
                self.state = self.state_next
        elif self.state == 2:
            # check for release
            if not self.flag_button_pressed:
                self.t_state = rospy.get_time() - self.t_button_press
                if self.t_state < self.dt_single_click_max:
                    self.click_count += 1

                    print self.click_count
                    # publish single/double click event
                    t_event = Int8()
                    t_event.data = self.click_count
                    self.button_event_publisher.publish(t_event)

                    if self.click_count == 1:
                        # switch to next state
                        self.state = 1
                        self.state_next = 3
                        self.t_button_release = rospy.get_time()
                    else:
                        # go back to button not pressed
                        self.click_count = 0
                        self.state = 0
                else:
                    # go back to button not pressed
                    self.click_count = 0
                    self.state = 0
        elif self.state == 3:
            # check for timer and click
            self.t_state = rospy.get_time() - self.t_button_release
            if self.t_state >= self.dt_double_click_max:
                # go back state 0
                self.click_count = 0
                self.state = 0

            if self.flag_button_pressed:
                # go to debounce
                self.state = 1
                self.state_next = 2
                self.t_button_press = rospy.get_time()

if __name__ == "__main__":
    # initialize node
    rospy.init_node("button_event_detector")

    # create a simple follower object
    detector = ButtonDetector()

    # spin
    rospy.spin()
