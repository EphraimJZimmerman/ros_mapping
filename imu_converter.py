#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion


class ImuConverter():
    def __init__(self):
        self.imu_sub = rospy.Subscriber(
            '/imu/data', Imu, callback=self.imu_cb)

    def imu_cb(self, msg):
        orientations = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientations)
        print(yaw)


if __name__ == '__main__':
    rospy.init_node('imu_converter')
    imu_converter = ImuConverter()
    rospy.spin()
