#! /usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import LaserScan


class ObstacleAvoider():
    def __init__(self):
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.ranges = None
        self.lidar_ranges = None  # Should be a dict
        print("inited")

    def scan_cb(self, msg):
        # Make lidar data independent of type of lidar
        min_angle = msg.angle_min
        max_angle = msg.angle_max
        start_angle = abs(max_angle) - abs(min_angle)
        # end_angle = abs(min_angle) + abs(max_angle)
        angle_increment = round(msg.angle_increment * 180.0/math.pi, 1)

        ranges = [i if (i > msg.range_min
                  and i < msg.range_max) else -1
                  for i in msg.ranges]
        angles = [(round(start_angle * (180/math.pi), 1) + angle_increment*i)
                  for i in range(0, len(ranges))]
        result = {}
        for i in range(0, len(angles)):
            result[angles[i]] = ranges[i]
        self.lidar_ranges = result
        print(str(self.lidar_ranges))
        # print(angle_increment)


if __name__ == '__main__':
    rospy.init_node('obstacle_avoider')
    obstacle_avoider = ObstacleAvoider()
    rospy.spin()
