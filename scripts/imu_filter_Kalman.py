#!/usr/bin/env python

import rospy
import numpy as np
from tf import transformations

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu

# some global constants
Ts = 0.02
g0 = np.array([0, 0, 1.0])
h0 = np.array([0.90, -0.01, -0.33])


class OrientationEstimator:
    def __init__(self):
        # raw data
        self.gyro_data = Vector3()
        self.acc_data = Vector3()
        self.mag_data = Vector3()

        # filtered data
        self.imu_filtered = Imu()

        # flag that indicates whether data has been updated
        self.data_flag = 0x00

        # "load" covariances - currently just hard code
        self.sigma_g = 0.4 * np.pi / 180.0
        self.sigma_b = 0.1 * np.pi / 180.0
        self.sigma_a = 0.0025
        self.sigma_h = 0.005

        # initial conditions
        self.q = np.array([0, 0, 0, 1.0])
        self.b = np.array([0, 0, 0.0])

        self.Sigma = np.vstack([np.hstack([(Ts / 2.0) ** 2 * self.sigma_g ** 2 * np.eye(4), np.zeros((4, 3))]),
                                np.hstack([np.zeros((3, 4)), self.sigma_b ** 2 * Ts ** 2 * np.eye(3)])])
        self.Qt = np.vstack([np.hstack([self.sigma_a ** 2 * np.eye(3), np.zeros((3, 3))]),
                             np.hstack([np.zeros((3, 3)), self.sigma_h**2 * np.eye(3)])])

        # subscriber and publisher
        self.gyro_data_sub = rospy.Subscriber("human_input/gyro_raw",
                                              Vector3, self.gyro_data_callback)
        self.acc_data_sub = rospy.Subscriber("human_input/acc_raw",
                                             Vector3, self.acc_data_callback)
        self.mag_data_sub = rospy.Subscriber("human_input/mag_raw",
                                             Vector3, self.mag_data_callback)

        self.imu_data_pub = rospy.Publisher("human_input/orientation",
                                            Imu, queue_size=1)

    def gyro_data_callback(self, msg):
        self.gyro_data = msg
        # print 'gyro data'
        self.data_flag |= 0x01

    def acc_data_callback(self, msg):
        self.acc_data = msg
        # print 'acc data'
        self.data_flag |= 0x02

    def mag_data_callback(self, msg):
        self.mag_data = msg
        # print 'mag data'
        self.data_flag |= 0x04

    @staticmethod
    def cross_mat(a):
        a_cross = np.array([[0, -a[2], a[1]],
                            [a[2], 0, -a[0]],
                            [-a[1], a[0], 0]])
        return a_cross

    def wait_for_data(self):
        while (self.data_flag != 0x07) and (not rospy.is_shutdown()):
            rospy.sleep(0.001)
            pass

    def get_init_orientation(self):
        acc_avg = np.array([0, 0, 0], dtype=np.float32)
        mag_avg = np.array([0, 0, 0], dtype=np.float32)

        num_init_data = 0

        # averaging for 1 second
        while num_init_data < 50:
            self.wait_for_data()
            self.data_flag = 0x00

            acc_avg[0] += self.acc_data.x
            acc_avg[1] += self.acc_data.y
            acc_avg[2] += self.acc_data.z
            mag_avg[0] += self.mag_data.x
            mag_avg[1] += self.mag_data.y
            mag_avg[2] += self.mag_data.z
            num_init_data += 1

        acc_avg /= num_init_data
        mag_avg /= num_init_data

        # convert to Euler angles
        roll = np.arctan2(acc_avg[1], acc_avg[2])
        pitch = np.arctan(-acc_avg[0] / (acc_avg[1] * np.sin(roll) + acc_avg[2] * np.cos(roll)))
        yaw = np.arctan2(mag_avg[2] * np.sin(roll) - mag_avg[1] * np.cos(roll),
                         mag_avg[0] * np.cos(pitch) + mag_avg[1] * np.sin(pitch) +
                         mag_avg[2] * np.sin(pitch) * np.cos(roll))
        print roll*180/np.pi, pitch*180/np.pi, yaw*180/np.pi

        # convert to quaternion
        self.q = transformations.quaternion_from_euler(roll, pitch, yaw)
        self.Sigma = np.vstack([np.hstack([(Ts/2.0)**2 * self.sigma_g**2 * np.eye(4), np.zeros((4, 3))]),
                                np.hstack([np.zeros((3, 4)), self.sigma_b**2 * Ts**2 * np.eye(3)])])

    def update(self):
        # clear the flag
        self.data_flag &= 0x00

        # convert from Vector3 to np.array
        omg = np.array([self.gyro_data.x, self.gyro_data.y, self.gyro_data.z]) * np.pi / 180.0
        acc = np.array([self.acc_data.x, self.acc_data.y, self.acc_data.z])
        mag = np.array([self.mag_data.x, self.mag_data.y, -self.mag_data.z])

        # parameters for process update
        omg_t = omg - self.b
        Omega = 0.5 * np.vstack([np.hstack([- self.cross_mat(omg_t), omg_t.reshape(3, 1)]),
                                  np.hstack([-omg_t, 0])])
        Xi = np.vstack([self.q[3] * np.eye(3) - self.cross_mat(self.q[0:3]), -self.q[0:3]])
        Gt = np.vstack([np.hstack([np.eye(4) + Ts * Omega, -Ts/2.0 * Xi]),
                        np.hstack([np.zeros((3, 4)), np.eye(3)])])
        Rt = np.vstack([np.hstack([(Ts/2.0)**2 * self.sigma_g**2 * Xi.dot(Xi.T), np.zeros((4, 3))]),
                        np.hstack([np.zeros((3, 4)), self.sigma_b**2 * Ts**2 * np.eye(3)])])

        # process update
        q_bar = (np.eye(4) + Ts * Omega).dot(self.q)
        b_bar = self.b
        Sigma_bar = Gt.dot(self.Sigma.dot(Gt.T)) + Rt

        # parameters for measurement update
        qx = self.cross_mat(self.q[0:3])
        gx = self.cross_mat(g0)
        hx = self.cross_mat(h0)

        Rq = np.eye(3) - 2 * self.q[3] * qx + 2 * qx.dot(qx)
        hz = np.hstack([Rq.dot(g0), Rq.dot(h0)])

        # print hz
        # print np.hstack([acc, mag])

        dq_a = np.hstack([2 * self.q[3] * gx + 2 * (gx.dot(qx) - 2 * qx.dot(gx)), -2 * (qx.dot(g0)).reshape(3, 1)])
        dq_h = np.hstack([2 * self.q[3] * hx + 2 * (hx.dot(qx) - 2 * qx.dot(hx)), -2 * (qx.dot(h0)).reshape(3, 1)])
        Ht = np.vstack([np.hstack([dq_a, np.zeros((3, 3))]),
                        np.hstack([dq_h, np.zeros((3, 3))])])

        # kalman gain
        # Kt = np.linalg.solve(Ht.dot(Sigma_bar.dot(Ht.T)) + self.Qt, Ht.dot(Sigma_bar.T))
        # Kt = Kt.T
        Kt = (Sigma_bar.dot(Ht.T)).dot(np.linalg.inv(Ht.dot(Sigma_bar.dot(Ht.T)) + self.Qt))

        # measurement update
        x = np.hstack([q_bar, b_bar]) + Kt.dot(np.hstack([acc, mag]) - hz)
        self.q = x[0:4] / np.linalg.norm(x[0:4])
        self.b = x[4:7]
        self.Sigma = (np.eye(7) - Kt.dot(Ht)).dot(Sigma_bar)

        # update imu data and publish
        self.imu_filtered.orientation.x = self.q[0]
        self.imu_filtered.orientation.y = self.q[1]
        self.imu_filtered.orientation.z = self.q[2]
        self.imu_filtered.orientation.w = self.q[3]

        self.imu_filtered.linear_acceleration = self.acc_data
        self.imu_filtered.angular_velocity = self.gyro_data

        # print filtered orientation

        euler = transformations.euler_from_quaternion(self.q, axes='sxyz')
        print euler[0]/np.pi*180, euler[1]/np.pi*180, euler[2]/np.pi*180


if __name__ == "__main__":
    rospy.init_node("IMU_kalman_filter")
    estimator = OrientationEstimator()

    # get initial pose first
    estimator.get_init_orientation()

    # loop rate 50Hz
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        estimator.wait_for_data()
        estimator.update()
        rate.sleep()
