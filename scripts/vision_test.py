#!/usr/bin/env python

import rospy
import cv2, cv_bridge
import numpy as np
import copy
import matplotlib.pyplot as plt

import fitEllipse

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Vector3
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
        self.vel2d_pub = rospy.Publisher("tracking/human_vel2d", Vector3, queue_size=1)
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
        self.vel_filter_alpha = rospy.get_param("~vel_filter_alpha", 0.3)
        self.dT = rospy.get_param("~dT", 0.1)

        # "focal length"
        self.fw = float(self.width) / 2.0 / np.tan(self.fov / 2.0)
        self.fh = float(self.height) / 2.0 / np.tan(self.fov / 2.0)

        # number of points for fitting
        self.num_fitting_point = 500

        self.rgb_image = np.zeros((self.height, self.width, 3), np.uint8)
        self.depth_image = np.zeros((self.height, self.width), np.uint16)
        self.mask = np.zeros((self.height, self.width), np.uint8)

        self.pos2d = Pose2D()
        self.pos2d_old = Pose2D()
        self.vel2d = Vector3()
        self.vel2d_old = Vector3()
        self.status = "Lost"

        # variables for fitting ellipse
        self.flag_first_fit = True
        self.ellipse_param = [0.0, 0.0, 0.0, 200, 150]

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

    def report_lost(self):
        self.status = "Lost"
        self.vel2d = Vector3()
        self.pos2d_pub.publish(self.pos2d)
        self.vel2d_pub.publish(self.vel2d)
        self.status_pub.publish(self.status)

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
            self.report_lost()
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
            self.report_lost()
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
            self.report_lost()
            return

        # calculate the center position of the blob
        M = cv2.moments(self.mask)
        if M['m00'] > 0:
            cx = M['m10'] / M['m00']
        else:
            cx = 0200

        self.pos2d.x = float(cx - self.width / 2) * avg_depth / self.fw
        self.pos2d.y = avg_depth

        # find the orientation
        row_start = 0  # max(int(M['m01'] / M['m00']) - 200, 0)
        row_end = min(int(M['m01'] / M['m00']) + 0, self.height)

        # find the non-zero points within the range
        nonzero_id = np.nonzero(self.mask[row_start:row_end, :] * self.depth_image[row_start:row_end, :])
        num_nonzero = np.size(nonzero_id, 1)

        # sample the non-zero points
        sample_step = int(num_nonzero / self.num_fitting_point) + 1
        sample_id = np.arange(0, num_nonzero, sample_step)

        # xx = nonzero_id[0][sample_id]
        # yy = nonzero_id[1][sample_id]
        sample_depth = self.depth_image[(nonzero_id[0][sample_id] + row_start, nonzero_id[1][sample_id])]
        sample_x = (nonzero_id[1][sample_id] - (self.width / 2)) * sample_depth / self.fw
        num_sample = np.size(sample_x)

        # # fit a ellipse
        # x_mean = np.mean(sample_x)
        # depth_mean = np.mean(sample_depth)
        #
        # ell = fitEllipse.fitEllipse((sample_x - x_mean) / 1000.0, (sample_depth - depth_mean) / 1000.0)
        # ell_center = fitEllipse.ellipse_center(ell) * 1000
        # phi = fitEllipse.ellipse_angle_of_rotation2(ell)
        # axes = fitEllipse.ellipse_axis_length(ell) * 1000

        # print ell_center, phi, axes

        # # fit a line
        # X = np.vstack((sample_x, sample_depth))
        # b = np.ones((num_sample, 1), np.float32)
        # a = np.dot(np.linalg.pinv(X.transpose()), b)
        # self.pos2d.theta = np.arctan2(a[0], a[1]) * 180 / 3.14159

        # # find first principal component
        X = np.vstack((sample_x, sample_depth))
        # X = X.transpose()
        #
        # X_mean = X.mean(axis=0)
        # X = X - X_mean
        # # print np.dot(X.T, X)
        #
        # eigenvectors, eigenvalues, V = np.linalg.svd(X, full_matrices=False)
        # pc1 = V[0]
        # pc1 /= np.linalg.norm(pc1)

        # fit ellipse with new method
        # if self.flag_first_fit:
        #     self.flag_first_fit = False
        #     self.ellipse_param[0] = np.mean(sample_x)
        #     self.ellipse_param[1] = np.mean(sample_depth)

        self.ellipse_param[0] = np.mean(sample_x)
        self.ellipse_param[1] = np.mean(sample_depth) + 150

        self.ellipse_param = fitEllipse.fit_ellipse(X.transpose(), self.ellipse_param, 2, 0.05, [100, 100, 0.2])

        # update the figure
        plt.clf()
        plt.scatter(sample_x, sample_depth)
        plt.axis([-500, 500, 500, 1500])

        # a, b = axes
        # R = np.arange(0, 2*np.pi, 0.1)
        # xx = ell_center[0] + x_mean + a * np.cos(R) * np.cos(phi) - b * np.sin(R) * np.sin(phi)
        # yy = ell_center[1] + depth_mean + a * np.cos(R) * np.sin(phi) + b * np.sin(R) * np.cos(phi)
        # plt.plot(xx, yy, color='red')

        # xx = np.arange(-300, 300, 10)
        # yy = (1 - a[0] * xx) / a[1]
        # plt.plot(xx, yy, color='red')

        # xx = np.arange(-200, 200, 2) * pc1[0] + X_mean[0]
        # yy = np.arange(-200, 200, 2) * pc1[1] + X_mean[1]
        # plt.plot(xx, yy, 'r')

        a = self.ellipse_param[3]
        b = self.ellipse_param[4]
        theta = self.ellipse_param[2]
        R = np.arange(0, 2*np.pi, 0.1)
        xx = self.ellipse_param[0] + a * np.cos(R) * np.cos(theta) - b * np.sin(R) * np.sin(theta)
        yy = self.ellipse_param[1] + a * np.cos(R) * np.sin(theta) + b * np.sin(R) * np.cos(theta)
        plt.plot(xx, yy, color='red')

        plt.pause(0.01)

        # calculate velocity based on discretization
        if self.status == "Find":
            # previous status is also find
            self.vel2d_old = self.vel2d
            self.vel2d.x = (1 - self.vel_filter_alpha) * self.vel2d_old.x + \
                           self.vel_filter_alpha * (self.pos2d.x - self.pos2d_old.x) / self.dT
            self.vel2d.y = (1 - self.vel_filter_alpha) * self.vel2d_old.y + \
                           self.vel_filter_alpha * (self.pos2d.y - self.pos2d_old.y) / self.dT
            self.vel2d.z = (1 - self.vel_filter_alpha) * self.vel2d_old.z + \
                           self.vel_filter_alpha * (self.pos2d.theta - self.pos2d_old.theta) / self.dT
            # print self.vel2d.y, self.pos2d_old.y, self.pos2d.y
        else:
            # previous lost, then simple set velocity to zero
            self.vel2d = Vector3()

        # update the old position
        self.pos2d_old = copy.deepcopy(self.pos2d)

        # publish the pos2d and velocity
        self.status = "Find"
        self.pos2d_pub.publish(self.pos2d)
        self.vel2d_pub.publish(self.vel2d)
        self.status_pub.publish(self.status)
        # print self.pos2d


if __name__ == "__main__":
    rospy.init_node('vision_test')
    tracker = SimpleHumanTracker()

    rospy.sleep(1)
    rate = rospy.Rate(10)

    plt.ion()
    while not rospy.is_shutdown():
        tracker.process_image()
        rate.sleep()
