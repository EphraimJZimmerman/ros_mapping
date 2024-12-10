#! /usr/bin/python3
import numpy as np
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Vector3
import rospy
from collections import deque

# Calibration Params

OFFSETS = [-13, -33, -33]
SCALES = [1.083333, 0.962963, 0.962963]

# Median Filter Params
WINDOW_SIZE = 20

# Create buffers for median filter
x_buffer = deque(maxlen=WINDOW_SIZE)
y_buffer = deque(maxlen=WINDOW_SIZE)
z_buffer = deque(maxlen=WINDOW_SIZE)


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

    x_buffer.append(corrected_mag_data[0])
    y_buffer.append(corrected_mag_data[1])
    z_buffer.append(corrected_mag_data[2])

    filtered_data = (np.median(x_buffer), np.median(
        y_buffer), np.median(z_buffer))

    # Create a new message to publish the corrected data
    corrected_mag_msg = MagneticField()
    corrected_mag_msg.magnetic_field = Vector3(*filtered_data)
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
