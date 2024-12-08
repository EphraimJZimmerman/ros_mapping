#!/usr/bin/env python

import rospy
from sensor_msgs.msg import MagneticField
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
import pickle

# List to store magnetometer data
mag_x = []
mag_y = []
mag_z = []

# ROS Subscriber callback to collect magnetometer data


def mag_callback(msg):
    global mag_x, mag_y, mag_z

    # Extract magnetometer data from the ROS message
    mag_x.append(msg.magnetic_field.x)
    mag_y.append(msg.magnetic_field.y)
    mag_z.append(msg.magnetic_field.z)


def get_magnetometer_data():
    # Initialize ROS node
    rospy.init_node('mag_calibration_node', anonymous=True)

    # Subscribe to the magnetometer topic (change topic as needed)
    rospy.Subscriber('/imu/mag', MagneticField, mag_callback)

    # Keep collecting data for a few seconds
    rospy.sleep(20)  # Collect data for 5 seconds (adjust as necessary)

    # Return collected data as numpy arrays
    return np.array(mag_x), np.array(mag_y), np.array(mag_z)


def plot_magnetometer_data(x, y, z):
    plt.figure(figsize=(10, 8))
    plt.scatter(x, y, c='blue', label='Magnetometer Data (XY)', alpha=0.5)
    plt.title("Magnetometer Calibration")
    plt.xlabel('Magnetometer X')
    plt.ylabel('Magnetometer Y')
    plt.grid(True)
    plt.legend(loc='best')
    plt.show()


def ellipse_residuals(params, x, y):
    # Ellipse equation: (x/a)^2 + (y/b)^2 = 1
    a, b, h, k, angle = params
    x_rot = (x - h) * np.cos(angle) + (y - k) * np.sin(angle)
    y_rot = -(x - h) * np.sin(angle) + (y - k) * np.cos(angle)
    return (x_rot / a)**2 + (y_rot / b)**2 - 1


def calibrate_ellipse(x, y):
    # Initial guess for ellipse parameters: a, b, h, k, angle
    initial_guess = [np.max(x), np.max(y), 0, 0, 0]
    result = least_squares(ellipse_residuals, initial_guess, args=(x, y))
    return result.x


def save_calibration_params(params):
    # Save calibration parameters to a file
    with open('magnetometer_calibration.pickle', 'wb') as f:
        pickle.dump(params, f)


def load_calibration_params():
    # Load calibration parameters from a file
    try:
        with open('magnetometer_calibration.pickle', 'rb') as f:
            return pickle.load(f)
    except FileNotFoundError:
        print("No calibration file found. Proceeding with uncalibrated data.")
        return None


def apply_calibration(x, y, params):
    a, b, h, k, angle = params
    # Apply the calibration to the data
    x_cal = (x - h) / a
    y_cal = (y - k) / b
    return x_cal, y_cal


if __name__ == '__main__':
    # Get magnetometer data
    x, y, z = get_magnetometer_data()

    # Plot the raw data (you'll see a scatter of points)
    plot_magnetometer_data(x, y, z)

    # Hard Iron Calibration (Removing bias)
    mag_center = np.mean([x, y, z], axis=1)
    x -= mag_center[0]
    y -= mag_center[1]
    z -= mag_center[2]

    # Soft Iron Calibration (Fitting ellipse)
    ellipse_params = calibrate_ellipse(x, y)
    print(
        f"Calibration parameters: a={ellipse_params[0]}, b={ellipse_params[1]}, h={ellipse_params[2]}, k={ellipse_params[3]}, angle={ellipse_params[4]}")

    # Save calibration parameters for future use
    save_calibration_params(ellipse_params)

    # Optionally, load the saved calibration parameters (if they exist)
    loaded_params = load_calibration_params()
    if loaded_params:
        print("Applying saved calibration parameters...")
        x_cal, y_cal = apply_calibration(x, y, loaded_params)
    else:
        # If no saved parameters, use the ones from current calibration
        x_cal, y_cal = apply_calibration(x, y, ellipse_params)

    # Plot the calibrated data
    plt.figure(figsize=(10, 8))
    plt.scatter(x_cal, y_cal, c='green',
                label='Calibrated Data (XY)', alpha=0.5)
    plt.title("Calibrated Magnetometer Data")
    plt.xlabel('Magnetometer X')
    plt.ylabel('Magnetometer Y')
    plt.grid(True)
    plt.legend(loc='best')
    plt.show()
