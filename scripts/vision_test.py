#!/usr/bin/env python

import rospy
import cv2, cv_bridge, numpy
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

        # initialize subscriber and publisher
        self.rgb_image_sub = rospy.Subscriber("camera/rgb/image_raw",
                                              Image, self.rgb_image_callback)
        self.depth_image_sub = rospy.Subscriber("camera/depth/image_raw",
                                                Image, self.depth_image_callback)
        self.pos2d_pub = rospy.Publisher("tracking/human_pos2d", Pose2D, queue_size=1)
        self.status_pub = rospy.Publisher("tracking/status", String, queue_size=1)

        # initialize variables
        self.width = 640
        self.height = 480
        self.fov = 57 * 3.1415926 / 180
        self.search_width = 20
        self.range_lost_th = 10

        self.rgb_image = numpy.zeros((self.height, self.width, 3), numpy.uint8)
        self.depth_image = numpy.zeros((self.height, self.width), numpy.float32)
        self.mask = numpy.zeros((self.height, self.width), numpy.uint8)

        self.pos2d = Pose2D()
        self.status = "Lost"

    # rgb image callback
    def rgb_image_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2HSV)

        # range for yellow object
        lower_yellow = numpy.array([20, 100, 100])
        upper_yellow = numpy.array([30, 255, 255])

        # threshold image
        self.mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        # res = cv2.bitwise_and(self.rgb_image, self.rgb_image, mask=mask)
        # cv2.imshow("window", res)
        # cv2.waitKey(3)

    # depth image callback
    def depth_image_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # cv2.imshow("window2", self.depth_image)
        # cv2.waitKey(3)

    def process_image(self):
        # only search for points close to the center line
        self.mask[0:(self.height / 2 - self.search_width), 0:self.width] = 0
        self.mask[(self.height / 2 + self.search_width):self.height, 0:self.width] = 0

        # apply mask to the depth sensor data
        self.depth_image = self.depth_image * self.mask / 255
        avg_depth = self.depth_image[(self.height/2 - self.search_width):(self.height/2 +
                                                                          self.search_width), 0:self.width].mean(axis=0)
        x = numpy.linspace(0, self.width-1, num=self.width)

        # correct the x coordinate based on FOV and depth
        x -= 0.5 * self.width
        x = x * 2*avg_depth*numpy.tan(self.fov/2.0) * (1.0/self.width)

        # find the range
        left = 0
        right = 0
        for i in range(0, self.width-1):
            if numpy.isfinite(avg_depth[i]) and avg_depth[i] > 0:
                left = i
                break
        for i in range(self.width-1, 0, -1):
            if numpy.isfinite(avg_depth[i]) and avg_depth[i] > 0:
                right = i
                break

        # return and report lost if the range is too small
        if right - left < self.range_lost_th:
            self.status = "Lost"
            self.pos2d_pub.publish(self.pos2d)
            self.status_pub.publish(self.status)
            return

        self.status = "Find"

        # moving average smoothing and find minimum
        d_min = 1e6
        x_min = 0
        smooth_depth = numpy.zeros(self.width, numpy.float32)
        for i in range(left+1, right-1):
            smooth_depth[i] = (avg_depth[i]+avg_depth[i-1]+avg_depth[i+1]) / 3.0
            if d_min > smooth_depth[i]:
                d_min = smooth_depth[i]
                x_min = i

        # find orientation, assuming that the long edge is always visible
        l1 = numpy.sqrt((x[left]-x[x_min])**2 + (avg_depth[left]-avg_depth[x_min]))
        l2 = numpy.sqrt((x[right]-x[x_min])**2 + (avg_depth[right]-avg_depth[x_min]))
        if l1 > l2:
            # long edge is to the left
            X = numpy.zeros((x_min-left, 2), numpy.float32)
            b = numpy.ones((x_min-left, 1), numpy.float32)
            X[:, 0] = x[left:x_min]
            X[:, 1] = avg_depth[left:x_min]
            a = numpy.dot(numpy.linalg.pinv(X), b)

            self.pos2d.theta = numpy.arctan2(a[0], a[1])
            self.pos2d.x = numpy.mean(x[left:x_min])
            self.pos2d.y = numpy.mean(avg_depth[left:x_min])
        else:
            # long edge is to the right
            X = numpy.zeros((right-x_min, 2), numpy.float32)
            b = numpy.ones((right-x_min, 1), numpy.float32)
            X[:, 0] = x[x_min:right]
            X[:, 1] = avg_depth[x_min:right]
            a = numpy.dot(numpy.linalg.pinv(X), b)

            self.pos2d.theta = numpy.arctan2(a[0], a[1])
            self.pos2d.x = numpy.mean(x[x_min:right])
            self.pos2d.y = numpy.mean(avg_depth[x_min:right])

        # publish the pos2d
        self.pos2d_pub.publish(self.pos2d)
        self.status_pub.publish(self.status)
        # print self.pos2d

        # plt.plot(x, avg_depth, linestyle=':')
        # plt.scatter(x[x_min], avg_depth[x_min], color='red', s=2)
        # plt.show()


if __name__ == "__main__":
    rospy.init_node('vision_test')
    tracker1 = SimpleHumanTracker()

    rospy.sleep(1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        tracker1.process_image()
        rate.sleep()
