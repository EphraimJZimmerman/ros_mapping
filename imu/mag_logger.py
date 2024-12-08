#!/usr/bin/env python
import rospy
from sensor_msgs.msg import MagneticField
from lino_msgs.msg import Imu
import math
import matplotlib as mpl


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
        self.mag_x = []
        self.mag_y = []

        while rospy.is_shutdown is False and self.Bx == 0:
            continue
        while rospy.is_shutdown() is False:
            # print(f" Mag : {self.Bx}, {self.By} || Imu: {self.e}, {self.n}")
            print(f"{self.Bx}, {self.By}")
            rospy.Rate(30).sleep()
            # self.mag_x.append(self.Bx)
            # self.mag_y.append(self.By)

    def graph(self):
        # Create a 2D scatter plot
        mpl.figure()
        mag_x = self.mag_x
        mag_y = self.mag_y
        mpl.scatter(mag_x, mag_y, c='b', label='Magnetometer Data')
        # Add reference line for y=0
        mpl.axhline(0, color='black', linewidth=0.5)
        # Add reference line for x=0
        mpl.axvline(0, color='black', linewidth=0.5)

        # Label the axes
        mpl.xlabel('Magnetometer X')
        mpl.ylabel('Magnetometer Y')
        mpl.title('2D Magnetometer Data (X-Y Plane)')
        mpl.legend()
        mpl.grid()
        mpl.axis('equal')  # Ensure aspect ratio is 1:1 for proper analysis

        # Show the plot
        mpl.show()


if __name__ == '__main__':
    try:
        MagneticComparer().run()
    except rospy.ROSInterruptException:
        pass
