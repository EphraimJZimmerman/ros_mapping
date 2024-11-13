#!/usr/bin/env python

import serial
import time

# Replace '/dev/tty.usbmodem1101' with your actual port
SERIAL_PORT = 'COM3'
BAUD_RATE = 9600


def parse_gprmc(data):
    parts = data.split(',')
    if parts[0] == '$GNRMC' and len(parts) > 9:
        time_utc = parts[1]
        status = parts[2]
        lat = parts[3]
        lat_dir = parts[4]
        lon = parts[5]
        lon_dir = parts[6]
        speed = parts[7]
        course = parts[8]
        date = parts[9]

        lat = convert_to_decimal_degrees(lat)
        lon = convert_to_decimal_degrees(lon)

        print(f"Time (UTC): {time_utc}")
        print(f"Status: {status}")
        print(f"Latitude: {lat} {lat_dir}")
        print(f"Longitude: {lon} {lon_dir}")
        print(f"Speed: {speed} knots")
        print(f"Course: {course} degrees")
        print(f"Date: {date}")
        print("-----")


def convert_to_decimal_degrees(coord):
    coord_multiplier = 1
    if coord[0] == '0':
        coord = coord[1:]
        coord_multiplier = -1

    return (float(coord[:2]) + float(coord[2:]) / 60.0) * coord_multiplier


def main():
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            print("Connected to GPS...")
            time.sleep(2)  # Wait for the GPS to warm up

            while True:
                line = ser.readline().decode('ascii', errors='ignore').strip()
                if line.startswith('$GNRMC'):
                    parse_gprmc(line)

    except serial.SerialException as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("Program terminated by user.")


if __name__ == '__main__':
    main()
