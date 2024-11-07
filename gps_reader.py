# /usr/bin/env python3
import serial
import time
import rospy
from sensor_msgs.msg import NavSatFix

# Replace '/dev/tty.usbmodem1101' with your actual port
SERIAL_PORT = 'COM3'
BAUD_RATE = 9600


class GPSReader():
    def __init__(self):
        self.gps_coord_pub = rospy.Publisher('/gps', NavSatFix, queue_size=1)

    def parse_gprmc(self, data):
        parts = data.split(',')
        if parts[0] == '$GNRMC' and len(parts) > 9:
            # I commented out the stuff we don't need for now, but might want to add to the message later
            # time_utc = parts[1]
            # status = parts[2]
            lat = parts[3]
            # lat_dir = parts[4]
            lon = parts[5]
            # lon_dir = parts[6]
            # speed = parts[7]
            # course = parts[8]
            # date = parts[9]

            gps_msg = NavSatFix()
            gps_msg.latitude = self.convert_to_decimal_degrees(lat)
            gps_msg.longitude = self.convert_to_decimal_degrees(lon)
            self.gps_coord_pub.publish(gps_msg)

            # print(f"Time (UTC): {time_utc}")
            # print(f"Status: {status}")
            # print(f"Latitude: {lat} {lat_dir}")
            # print(f"Longitude: {lon} {lon_dir}")
            # print(f"Speed: {speed} knots")
            # print(f"Course: {course} degrees")
            # print(f"Date: {date}")
            # print("-----")

    def convert_to_decimal_degrees(self, coord):
        coord_multiplier = 1
        if coord[0] == '0':
            coord = coord[1:]
            coord_multiplier = -1

        return (float(coord[:2]) + float(coord[2:]) / 60.0) * coord_multiplier

    def run(self):
        try:
            with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
                rospy.loginfo("Connected to GPS...")
                time.sleep(2)  # Wait for the GPS to warm up

                while rospy.is_shutdown() is not False:
                    line = ser.readline().decode('ascii', errors='ignore').strip()
                    if line.startswith('$GNRMC'):
                        self.parse_gprmc(line)

        except serial.SerialException as e:
            print(f"Error: {e}")
        except KeyboardInterrupt:
            print("Program terminated by user.")

        rospy.loginfo("Program terminated by user")


if __name__ == '__main__':
    rospy.init_node('gps_reader')
    gps_reader = GPSReader()
    gps_reader.run()
