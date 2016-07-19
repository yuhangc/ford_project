#!/usr/bin/env python

import rospy
import cv2, cv_bridge
import numpy as np
# import matplotlib.pyplot as plt
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
        self.search_width = rospy.get_param("~search_width", 20)
        self.range_lost_th = rospy.get_param("~range_lost_threshold", 10)
        self.depth_scale = rospy.get_param("~depth_scale", 1000.0)

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

        # only search for points close to the center line
        self.mask[0:(self.height / 2 - self.search_width), 0:self.width] = 0
        self.mask[(self.height / 2 + self.search_width):self.height, 0:self.width] = 0

        # filter the mask with erosion and close filter
        self.mask = cv2.erode(self.mask, self.kernel_erosion, iterations=1)
        self.mask = cv2.morphologyEx(self.mask, cv2.MORPH_CLOSE, self.kernel_closing)

        # cv2.imshow("window", self.mask)
        # cv2.waitKey(3)

        # apply mask to the depth sensor data
        self.depth_image = self.depth_image * (self.mask / 255)
        avg_depth = self.depth_image[(self.height/2 - self.search_width):(self.height/2 +
                                                                          self.search_width), 0:self.width].mean(axis=0)
        x = np.linspace(0, self.width-1, num=self.width)
        x -= 0.5 * self.width

        # find the range
        left = 0
        right = 0
        for i in range(0, self.width-1):
            if np.isfinite(avg_depth[i]) and avg_depth[i] > 0:
                left = i
                break
        for i in range(self.width-1, 0, -1):
            if np.isfinite(avg_depth[i]) and avg_depth[i] > 0:
                right = i
                break

        # return and report lost if the range is too small
        if right - left < self.range_lost_th:
            self.status = "Lost"
            self.pos2d_pub.publish(self.pos2d)
            self.status_pub.publish(self.status)
            return

        self.status = "Find"

        # find orientation, assuming that the long edge is always visible
        X = np.zeros((right-left, 2), np.float32)
        b = np.ones((right-left, 1), np.float32)
        X[:, 0] = x[left:right]
        X[:, 1] = avg_depth[left:right]
        a = np.dot(np.linalg.pinv(X), b)

        self.pos2d.theta = np.arctan2(a[0], a[1])
        self.pos2d.x = np.mean(x[left:right])
        self.pos2d.y = np.mean(avg_depth[left:right]) / self.depth_scale

        # publish the pos2d
        self.pos2d_pub.publish(self.pos2d)
        self.status_pub.publish(self.status)
        # print self.pos2d

        # plt.plot(x, avg_depth, linestyle=':')
        # plt.scatter(x[x_min], avg_depth[x_min], color='red', s=2)
        # plt.show()


if __name__ == "__main__":
    rospy.init_node('vision_test')
    tracker = SimpleHumanTracker()

    rospy.sleep(1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        tracker.process_image()
        rate.sleep()
