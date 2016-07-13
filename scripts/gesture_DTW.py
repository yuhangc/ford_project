#!/usr/bin/env python

import rospy
import numpy as np

from collections import deque

from geometry_msgs.msg import Vector3
from std_msgs.msg import Int8

gesture_dict = {0: "backward",
                1: "forward",
                2: "turn_right/cw",
                3: "turn_left/ccw"}


class GestureDTW:
    def __init__(self):
        # set parameters
        self.num_gestures = rospy.get_param("~num_gestures", 4)
        self.max_window_size = rospy.get_param("~max_window_size", 45)
        self.dtw_window = rospy.get_param("~dtw_window", 5)

        self.gesture_temps = []
        self.gesture_len = []
        self.rej_thresh = []
        self.rej_corr = [1.5, 1.5, 1.5, 1.5]    # should load from file

        # obtain data file path through parameter
        data_file_path = rospy.get_param("~gesture_data_path", "../gesture_data")
        self.__load_data__(data_file_path)

        # acceleration data input
        self.acc_raw = np.array([0, 0, 0], dtype=np.float32)
        self.acc_filtered = deque(maxlen=self.max_window_size)
        self.acc_hist_len = 0

        # recognized gesture number
        self.gesture_id = Int8()

        # a flag that indicates that a gesture has been detected
        self.flag_gesture = False

        # a mini state machine for debouncing
        self.gesture_state = 0
        self.time_gesture_start = 0.0
        self.time_debounce = 0.5

        # subscriber and publisher
        self.acc_data_sub = rospy.Subscriber("human_input/acc_raw",
                                             Vector3, self.acc_data_callback)
        self.gesture_pub = rospy.Publisher("human_input/gesture",
                                           Int8, queue_size=1)

    def __load_data__(self, data_file_path):
        self.num_gestures = 4

        # open and read the threshold file
        data_file = open(data_file_path + "/rejection_threshold.txt", 'r')
        for g in range(0, self.num_gestures):
            thresh_data = data_file.readline()
            self.rej_thresh.append(float(thresh_data))

        # close the file
        data_file.close()

        # load the gesture template files
        for g in range(0, self.num_gestures):
            data_file = open(data_file_path + "/temp_gesture" + str(g) + ".txt", 'r')
            data_all = data_file.readlines()

            temp = np.zeros((len(data_all), 3), dtype=np.float32)
            temp_len = 0

            for data_point in data_all:
                temp[temp_len, :] = np.array(map(float, data_point.split(',')))
                temp_len += 1

            self.gesture_temps.append(temp)
            self.gesture_len.append(temp_len)

            # close the file
            data_file.close()

    def acc_data_callback(self, msg):
        self.acc_raw[0] = msg.x
        self.acc_raw[1] = msg.y
        self.acc_raw[2] = msg.z
        # print self.acc_raw

        # add data to acc history queue
        self.add_filtered_acc()

        # do gesture detection
        if self.gesture_state == 0:
            # do detection
            self.gesture_matching()

            if self.flag_gesture:
                # publish the gesture
                self.gesture_pub.publish(self.gesture_id)

                # reset the flag
                self.flag_gesture = False

                # switch to not detect state
                self.gesture_state = 1
                self.time_gesture_start = rospy.get_time()
        else:
            # no gesture detection, just check time
            if rospy.get_time() - self.time_gesture_start > self.time_debounce:
                self.gesture_state = 0

    def add_filtered_acc(self):
        # just a simple IIR filter
        if not self.acc_filtered:
            self.acc_filtered.append(self.acc_raw)
        else:
            acc_last = self.acc_filtered.pop()
            acc_new = 0.5 * acc_last + 0.5 * self.acc_raw

            self.acc_filtered.append(acc_last)
            self.acc_filtered.append(acc_new)

    def gesture_matching(self):
        gesture_score = []
        self.flag_gesture = False

        # only start to do gesture detection after having enough data
        if len(self.acc_filtered) < self.max_window_size:
            return

        # do DTW with each gesture template
        for g in range(0, self.num_gestures):
            W = 1e6 * np.ones((self.max_window_size+1, self.gesture_len[g]+1), dtype=np.float32)
            W[0, 0] = 0
            gesture = self.gesture_temps[g]

            for i in range(0, self.max_window_size):
                acc_i = self.acc_filtered.popleft()
                for j in range(max(0, i - self.dtw_window), min(self.gesture_len[g], i + self.dtw_window)):
                    W[i+1, j+1] = np.linalg.norm(acc_i - gesture[j, :]) + \
                                  min(W[i, j+1], W[i+1, j], W[i, j])

                self.acc_filtered.append(acc_i)

            gesture_score.append(W[self.max_window_size, self.gesture_len[g]] / self.gesture_len[g])
            if gesture_score[g] > self.rej_thresh[g] * self.rej_corr[g]:
                # gesture_score[g] = 1e6
                pass
            else:
                self.flag_gesture = True

        # print gesture_score
        # find the best match
        if self.flag_gesture:
            self.gesture_id = gesture_score.index(min(gesture_score))
            print gesture_dict[self.gesture_id]
            self.gesture_pub.publish(self.gesture_id)

if __name__ == "__main__":
    # initialize node and a gesture recognition object
    rospy.init_node("gesture_detector")
    gesture_rec = GestureDTW()

    # loop rate 50Hz
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        rate.sleep()
