#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math


class ImuConverter():
    def __init__(self):
        self.imu_sub = rospy.Subscriber(
            '/imu/data', Imu, callback=self.imu_cb)

        self.imu_yaw = 0
        self.direction = ""

    def imu_cb(self, msg):
        orientations = [msg.orientation.x, msg.orientation.y,
                        msg.orientation.z, msg.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientations)
        self.imu_yaw = (math.degrees(yaw) + 360) % 360
        if 45 <= self.imu_yaw < 135:
            self.direction = "South"
        elif 135 <= self.imu_yaw < 225:
            self.direction = "East"
        elif 225 <= self.imu_yaw < 315:
            self.direction = "North"
        else:
            self.direction = "West"

    def run(self):
        while not rospy.is_shutdown():
            print(
                f" Direction: {self.direction} ||| Angle: {self.imu_yaw : .2f}")


if __name__ == '__main__':
    rospy.init_node('imu_converter')
    imu_converter = ImuConverter()
    imu_converter.run()
    rospy.spin()
