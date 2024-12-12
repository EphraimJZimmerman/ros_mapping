#!/usr/bin/env python
import rospy
from sensor_msgs.msg import MagneticField
from lino_msgs.msg import Imu


class MagneticComparer():
    '''
        Logs raw magnetomter values to console in csv format
    '''

    def magnetic_callback(self, msg):
        self.mag_x = msg.magnetic_field.x
        self.mag_y = msg.magnetic_field.y
        self.mag_z = msg.magnetic_field.z

    def __init__(self):
        rospy.init_node('magnetic_orientation', anonymous=True)
        self.mag_sub = rospy.Subscriber(
            "/imu/mag_corrected", MagneticField, self.magnetic_callback)
        self.imu_sub = rospy.Subscriber("/raw_imu", Imu, self.imu_cb)

        self.mag_x = 0
        self.mag_y = 0
        self.mag_z = 0

    def run(self):
        print("x,y,z")

        while rospy.is_shutdown is False and self.mag_x == 0:
            continue
        while rospy.is_shutdown() is False:
            print(f"{self.mag_x}, {self.mag_y}, {self.mag_z}")
            rospy.Rate(30).sleep()


if __name__ == '__main__':
    try:
        MagneticComparer().run()
    except rospy.ROSInterruptException:
        pass
