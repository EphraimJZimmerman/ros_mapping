#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float64


class ImuConverter():
    def __init__(self):
        self.imu_sub = rospy.Subscriber(
            '/imu', Imu, callback=self.imu_cb)

        self.imu_sub = rospy.Subscriber(
            '/odom', Odometry, callback=self.odom_cb)

        self.imu_pub = rospy.Publisher('/imu/yaw', Float64, queue_size=1)

        self.odom_yaw = 0
        self.imu_yaw = 0

    def imu_cb(self, msg):
        orientations = [msg.orientation.x, msg.orientation.y,
                        msg.orientation.z, msg.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientations)
        self.imu_yaw = yaw
        self.imu_pub.publish(self.imu_yaw)

    def odom_cb(self, msg):
        orientation = msg.pose.pose.orientation
        orientations = [orientation.x, orientation.y,
                        orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientations)
        self.odom_yaw = yaw

    def run(self):
        while not rospy.is_shutdown():
            print(
                f" Odom_yaw: {self.odom_yaw : .2f} ||| Imu yaw: {self.imu_yaw : .2f}")


if __name__ == '__main__':
    rospy.init_node('imu_converter')
    imu_converter = ImuConverter()
    imu_converter.run()
    rospy.spin()
