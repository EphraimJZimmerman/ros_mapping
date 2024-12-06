#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
import tf.transformations
import math


def imu_callback(msg):
    # Extract quaternion from the IMU message
    orientation_q = msg.orientation
    quaternion = (
        orientation_q.x,
        orientation_q.y,
        orientation_q.z,
        orientation_q.w
    )

    # Convert quaternion to Euler angles
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)

    # Convert yaw from radians to degrees
    yaw_degrees = math.degrees(yaw)

    # Normalize yaw to [0, 360) degrees
    yaw_degrees = (yaw_degrees + 360) % 360

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


def imu_orientation():
    rospy.init_node('imu_orientation', anonymous=True)
    rospy.Subscriber("/imu/data", Imu, imu_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        imu_orientation()
    except rospy.ROSInterruptException:
        pass
