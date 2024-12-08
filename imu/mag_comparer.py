#!/usr/bin/env python
import rospy
from sensor_msgs.msg import MagneticField
from lino_msgs.msg import Imu
import math


class MagneticComparer():

    def magnetic_callback(self, msg):
        # Extract magnetic field components in ENU convention
        self.Bx = msg.magnetic_field.x  # East component
        self.By = msg.magnetic_field.y  # North component

    def imu_cb(self, msg):
        self.e = msg.magnetic_field.x
        self.n = msg.magnetic_field.y

    def __init__(self):
        rospy.init_node('magnetic_orientation', anonymous=True)
        self.mag_sub = rospy.Subscriber(
            "/imu/mag", MagneticField, self.magnetic_callback)
        self.imu_sub = rospy.Subscriber("/raw_imu", Imu, self.imu_cb)

        self.Bx = 0
        self.By = 0
        self.e = 0
        self.n = 0

    def run(self):
        while rospy.is_shutdown() is False:
            print(f" Mag : {self.Bx}, {self.By} || Imu: {self.e}, {self.n}")


if __name__ == '__main__':
    try:
        MagneticComparer().run()
    except rospy.ROSInterruptException:
        pass
