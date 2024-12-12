#! /usr/bin/python3
import numpy as np
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Vector3
import rospy
from collections import deque

'''Calibration Params'''
# Replace with the offset values you get from calibration_plotter.py
OFFSETS = [-13, -33, -33]
# Replace with the scale values you get from calibration_plotter.py
SCALES = [1.083333, 0.962963, 0.962963]

''' Median Filter Params '''
# Adjust this to increase the buffer size for the median filter.
# Increasing will reduce noise, decreasing will increase noise.
WINDOW_SIZE = 7

# Create buffers for median filter
x_buffer = deque(maxlen=WINDOW_SIZE)
y_buffer = deque(maxlen=WINDOW_SIZE)
z_buffer = deque(maxlen=WINDOW_SIZE)


def apply_calibration(raw_mag_data, offsets, scales):
    '''
        Apply soft and hard iron correction to the raw data and return the 
        corrected data
    '''
    # Apply hard-iron correction
    corrected_data = np.array(raw_mag_data) - np.array(offsets)

    # Apply soft-iron correction
    corrected_data = corrected_data / np.array(scales)

    return corrected_data


def mag_callback(msg):
    '''
        ROS Subscriber callback to apply calibration and publish corrected data
    '''
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


# Run the actual node
rospy.init_node('magnetometer_calibration')
mag_pub = rospy.Publisher('/imu/mag_corrected', MagneticField, queue_size=10)
rospy.Subscriber('/imu/mag', MagneticField, mag_callback)
rospy.spin()
