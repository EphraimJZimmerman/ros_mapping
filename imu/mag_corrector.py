#! /usr/bin/python3
import numpy as np
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Vector3
import rospy

# Calibration Params
OFFSETS = [-11, -34, -34]
SCALES = [1.039216, 0.981481, 0.981481]

# OFFSETS = [-10, -31, -31]
# SCALES = [1.074074, 0.966667, 0.966667]


def apply_calibration(raw_mag_data, offsets, scales):
    # Apply hard-iron correction (subtract the offset)
    corrected_data = np.array(raw_mag_data) - np.array(offsets)

    # Apply soft-iron correction (matrix multiplication)
    corrected_data = corrected_data / np.array(scales)

    # Return the corrected data
    return corrected_data

# ROS Subscriber callback to apply calibration and publish corrected data


def mag_callback(msg):
    # Extract magnetometer data from the message
    raw_mag_data = np.array(
        [msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z])

    corrected_mag_data = apply_calibration(raw_mag_data, OFFSETS, SCALES)

    # Create a new message to publish the corrected data
    corrected_mag_msg = MagneticField()
    corrected_mag_msg.magnetic_field = Vector3(*corrected_mag_data)
    corrected_mag_msg.header = msg.header

    # Publish the corrected data
    mag_pub.publish(corrected_mag_msg)


# Initialize ROS node
rospy.init_node('magnetometer_calibration')

# Create a publisher for the corrected magnetometer data
mag_pub = rospy.Publisher('/imu/mag_corrected', MagneticField, queue_size=10)

# Subscribe to the original magnetometer topic
rospy.Subscriber('/imu/mag', MagneticField, mag_callback)

# Spin to keep the node running
rospy.spin()
