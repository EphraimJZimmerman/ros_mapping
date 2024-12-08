#!/usr/bin/env python
import numpy as np
import pickle
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Vector3
import rospy

# Function to load calibration parameters


def load_calibration_params():
    try:
        with open('magnetometer_calibration.pickle', 'rb') as f:
            return pickle.load(f)
    except FileNotFoundError:
        print("No calibration file found. Proceeding with uncalibrated data.")
        return None


def get_hard_soft_iron_params(ellipse_params):
    # Function to appldef get_hard_soft_iron_params(ellipse_params):
    a, b, h, k, angle = ellipse_params

    # Hard-Iron Offset (bias)
    # Assuming 2D calibration, set Z to 0 if not available
    hard_iron_offset = np.array([h, k, 0])

    # Soft-Iron Matrix (scaling)
    soft_iron_matrix = np.array([[1/(a**2), 0],
                                 [0, 1/(b**2)]])

    # Rotation matrix (due to the angle of the ellipse)
    rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)],
                                [np.sin(angle), np.cos(angle)]])

    # Apply the rotation to the soft-iron matrix (combine rotation and scaling)
    soft_iron_matrix_rotated = np.dot(
        np.dot(rotation_matrix, soft_iron_matrix), rotation_matrix.T)

    return hard_iron_offset, soft_iron_matrix_rotated


def apply_calibration(x, y, z, hard_iron_offset, soft_iron_matrix):
    # Apply hard-iron correction (subtract the offset)
    corrected_data = np.array([x, y, z]) - hard_iron_offset

    # Apply soft-iron correction (matrix multiplication)
    # Use only X, Y for 2D calibration
    corrected_data = np.dot(soft_iron_matrix, corrected_data[:2])

    # Return the corrected data (Z is unchanged for 2D calibration)
    return corrected_data[0], corrected_data[1], z

# ROS Subscriber callback to apply calibration and publish corrected data


def mag_callback(msg, calibration_params):
    # Extract magnetometer data from the message
    raw_mag_data = np.array(
        [msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z])

    # Apply the calibration if calibration parameters are available
    if calibration_params is not None:
        hard_iron_matrix, soft_iron_matrix = get_hard_soft_iron_params(
            calibration_params)
        corrected_mag_data = apply_calibration(
            raw_mag_data[0], raw_mag_data[1], raw_mag_data[2], hard_iron_matrix, soft_iron_matrix)
    else:
        corrected_mag_data = raw_mag_data  # No calibration, keep raw data

    # Create a new message to publish the corrected data
    corrected_mag_msg = MagneticField()
    corrected_mag_msg.magnetic_field = Vector3(*corrected_mag_data)

    # Publish the corrected data
    mag_pub.publish(corrected_mag_msg)


# Initialize ROS node
rospy.init_node('magnetometer_calibration')

# Load calibration parameters
calibration_params = load_calibration_params()

# Create a publisher for the corrected magnetometer data
mag_pub = rospy.Publisher('/imu/mag_corrected', MagneticField, queue_size=10)

# Subscribe to the original magnetometer topic
rospy.Subscriber('/imu/mag', MagneticField, mag_callback, calibration_params)

# Spin to keep the node running
rospy.spin()
