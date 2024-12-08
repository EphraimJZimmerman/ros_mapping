#!/usr/bin/env python
import rospy
from sensor_msgs.msg import MagneticField
from lino_msgs.msg import Imu
import math
import matplotlib as mpl


class MagneticComparer():

    def magnetic_callback(self, msg):
        # Extract magnetic field components in ENU convention
        self.mag_x = msg.magnetic_field.x  # East component
        self.mag_y = msg.magnetic_field.y  # North component
        self.mag_z = msg.magnetic_field.y  # North component

    def imu_cb(self, msg):
        self.e = msg.magnetic_field.x
        self.n = msg.magnetic_field.y

    def __init__(self):
        rospy.init_node('magnetic_orientation', anonymous=True)
        self.mag_sub = rospy.Subscriber(
            "/imu/mag", MagneticField, self.magnetic_callback)
        self.imu_sub = rospy.Subscriber("/raw_imu", Imu, self.imu_cb)

        self.mag_x = 0
        self.mag_y = 0
        self.mag_z = 0
        self.e = 0
        self.n = 0

    def run(self):
        self.mag_x = []
        self.mag_y = []
        self.mag_z = []
        print(f"x,y,z")

        while rospy.is_shutdown is False and self.mag_x == []:
            continue
        while rospy.is_shutdown() is False:
            # print(f" Mag : {self.mag_x}, {self.mag_y} || Imu: {self.e}, {self.n}")
            print(f"{self.mag_x}, {self.mag_y}, {self.mag_z}")
            rospy.Rate(30).sleep()
            # self.mag_x.append(self.mag_x)
            # self.mag_y.append(self.mag_y)


if __name__ == '__main__':
    try:
        MagneticComparer().run()
    except rospy.ROSInterruptException:
        pass
