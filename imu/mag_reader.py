#!/usr/bin/env python
import rospy
from sensor_msgs.msg import MagneticField
import math


def magnetic_callback(msg):
    # Extract magnetic field components in NED convention
    Bx = msg.magnetic_field.x  # North South component
    By = msg.magnetic_field.y  # East West component

    # Calculate the yaw angle (heading) in radians
    # yaw_radians = math.atan2(Bx, By)
    yaw_radians = math.atan2(-By, Bx)

    # Convert radians to degrees and normalize to [0, 360)
    yaw_degrees = (math.degrees(yaw_radians) + 360) % 360

    # Determine direction based on yaw
    if 45 <= yaw_degrees < 135:
        direction = "East"
    elif 135 <= yaw_degrees < 225:
        direction = "South"
    elif 225 <= yaw_degrees < 315:
        direction = "West"
    else:
        direction = "North"

    rospy.loginfo(f"Yaw: {yaw_degrees:.2f} degrees, Facing: {direction}")


def magnetic_orientation():
    rospy.init_node('magnetic_orientation', anonymous=True)
    rospy.Subscriber("/imu/mag_corrected", MagneticField, magnetic_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        magnetic_orientation()
    except rospy.ROSInterruptException:
        pass
