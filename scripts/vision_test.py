#!/usr/bin/env python

import rospy
import cv2, cv_bridge
import numpy as np
import matplotlib.pyplot as plt

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String


class SimpleHumanTracker:
    def __init__(self):
        # initialize open cv
        self.bridge = cv_bridge.CvBridge()
        # cv2.namedWindow("window", 1)
        # cv2.namedWindow("window2", 1)

        # image topic source
        rgb_image_source = rospy.get_param("~rgb_image_source", "camera/rgb/image_raw")
        depth_image_source = rospy.get_param("~depth_image_source", "camera/depth_registered/image_raw")

        # initialize subscriber and publisher
        self.rgb_image_sub = rospy.Subscriber(rgb_image_source,
                                              Image, self.rgb_image_callback)
        self.depth_image_sub = rospy.Subscriber(depth_image_source,
                                                Image, self.depth_image_callback)
        self.pos2d_pub = rospy.Publisher("tracking/human_pos2d", Pose2D, queue_size=1)
        self.status_pub = rospy.Publisher("tracking/status", String, queue_size=1)

        # initialize variables
        self.width = 640
        self.height = 480

        # kernels for filter
        self.kernel_erosion = np.ones((5, 5), np.uint8)
        self.kernel_closing = np.ones((8, 8), np.uint8)

        # flags that indicates data received
        self.flag_rgb_received = False
        self.flag_depth_received = False

        # get parameters
        self.fov = rospy.get_param("~field_of_view", 57) * 3.1415926 / 180
        self.range_lost_th = rospy.get_param("~range_lost_threshold", 10000)
        self.depth_scale = rospy.get_param("~depth_scale", 1000.0)
        self.height_cap = rospy.get_param("~height_cap", 20)
        self.width_cap = rospy.get_param("~width_cap", 10)
        self.depth_cap_min = rospy.get_param("~depth_cap_min", 0.3)

        # "focal length"
        self.fw = float(self.width) / 2.0 / np.tan(self.fov / 2.0)
        self.fh = float(self.height) / 2.0 / np.tan(self.fov / 2.0)

        # number of points for fitting
        self.num_fitting_point = 100

        self.rgb_image = np.zeros((self.height, self.width, 3), np.uint8)
        self.depth_image = np.zeros((self.height, self.width), np.uint16)
        self.mask = np.zeros((self.height, self.width), np.uint8)

        self.pos2d = Pose2D()
        self.status = "Lost"

    # rgb image callback
    def rgb_image_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2HSV)

        # range for yellow object
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])

        # threshold image
        self.mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        self.flag_rgb_received = True
        # res = cv2.bitwise_and(self.rgb_image, self.rgb_image, mask=self.mask)
        # cv2.imshow("window", res)
        # cv2.waitKey(3)

    # depth image callback
    def depth_image_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        self.flag_depth_received = True

        # cv2.imshow("window2", self.depth_image * 8)
        # cv2.waitKey(3)

    def process_image(self):
        # data syncing
        while not (self.flag_rgb_received and self.flag_depth_received):
            rospy.sleep(0.001)
        self.flag_rgb_received = False
        self.flag_depth_received = False

        # filter the mask with erosion and close filter
        self.mask = cv2.erode(self.mask, self.kernel_erosion, iterations=1)
        self.mask = cv2.morphologyEx(self.mask, cv2.MORPH_CLOSE, self.kernel_closing)

        # cap the boarder of the mask to prevent noise
        self.mask[0:self.height_cap, 0:self.width] = 0
        self.mask[0:self.height, 0:self.width_cap] = 0
        self.mask[0:self.height, (self.width - self.width_cap):self.width] = 0

        # find contours
        cnts, hierarchy = cv2.findContours(self.mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # return if no contours found
        if len(cnts) == 0:
            self.status = "Lost"
            self.pos2d_pub.publish(self.pos2d)
            self.status_pub.publish(self.status)
            return

        # only use the largest contour
        areas = np.zeros((1, len(cnts)))
        idx = 0
        for cnt in cnts:
            areas[0, idx] = cv2.contourArea(cnt)
            idx += 1

        # find the max area
        area = areas.max()
        idx = areas.argmax()

        # return and report lost if the range is too small
        if area < self.range_lost_th:
            self.status = "Lost"
            self.pos2d_pub.publish(self.pos2d)
            self.status_pub.publish(self.status)
            return

        # update the mask
        self.mask = np.zeros((self.height, self.width), np.uint8)
        cv2.drawContours(self.mask, cnts, idx, 255, -1)

        # cv2.imshow("window", self.mask)
        # cv2.waitKey(3)

        # calculate the average distance to the blob
        avg_depth = cv2.mean(self.depth_image, self.mask)
        avg_depth = avg_depth[0] / self.depth_scale
        # print avg_depth

        # if too close don't update since the depth sensor won't work
        if avg_depth <= self.depth_cap_min:
            self.status = "Lost"
            self.pos2d_pub.publish(self.pos2d)
            self.status_pub.publish(self.status)
            return

        # calculate the center position of the blob
        M = cv2.moments(self.mask)
        if M['m00'] > 0:
            cx = M['m10'] / M['m00']
        else:
            cx = 0

        self.status = "Find"

        self.pos2d.x = float(cx - self.width / 2) * avg_depth / self.fw
        self.pos2d.y = avg_depth

        # find the orientation
        row_start = max(int(M['m01'] / M['m00']) - 5, 0)
        row_end = min(int(M['m01'] / M['m00']) + 5, self.height)

        # find the non-zero points within the range
        temp = self.mask[row_start:row_end, :]
        nonzero_id = np.nonzero(self.mask[row_start:row_end, :])
        num_nonzero = np.size(nonzero_id, 1)

        # sample the non-zero points
        sample_step = int(num_nonzero / self.num_fitting_point) + 1
        sample_id = np.arange(0, num_nonzero, sample_step)

        # xx = nonzero_id[0][sample_id]
        # yy = nonzero_id[1][sample_id]
        sample_depth = self.depth_image[(nonzero_id[0][sample_id] + row_start, nonzero_id[1][sample_id])]
        sample_x = (nonzero_id[1][sample_id] - (self.width / 2)) * sample_depth / self.fw
        num_sample = np.size(sample_x)

        # plt.scatter(sample_x, sample_depth)

        sample_x_t = np.transpose(sample_x)
        # fit a line
        X = np.vstack((sample_x, sample_depth))
        b = np.ones((num_sample, 1), np.float32)
        a = np.dot(np.linalg.pinv(X.transpose()), b)

        self.pos2d.theta = np.arctan2(a[0], a[1]) * 180 / 3.14159

        # publish the pos2d
        self.pos2d_pub.publish(self.pos2d)
        self.status_pub.publish(self.status)
        # print self.pos2d


if __name__ == "__main__":
    rospy.init_node('vision_test')
    tracker = SimpleHumanTracker()

    rospy.sleep(1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        tracker.process_image()
        rate.sleep()
