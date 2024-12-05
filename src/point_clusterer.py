#! /usr/bin/env python3
import rospy
import math
import numpy as np
from sklearn.cluster import DBSCAN
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


# Ideas
# Do not need to actually wall follow around obstacles, just stop and turn until they are no longer within your front cone and go around them that way
class ObstacleClusterer:
    def __init__(self):
        # Subscribe to LaserScan messages
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)

        # Publisher for visualization markers
        self.marker_pub = rospy.Publisher(
            '/obstacle_markers', MarkerArray, queue_size=1)

        # Clustering parameters
        self.eps = 0.3  # Maximum distance between points in a cluster (meters)
        self.min_samples = 5  # Minimum points to form a cluster

        self.lidar_ranges = None
        print("Obstacle Clusterer initialized")

    def polar_to_cartesian(self, angle, range_val):
        """Convert polar coordinates to cartesian"""
        x = range_val * math.cos(angle)
        y = range_val * math.sin(angle)
        return [x, y]

    def create_cluster_marker(self, points, cluster_id):
        """Create a visualization marker for a cluster"""
        marker = Marker()
        marker.header.frame_id = "scan_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "clusters"
        marker.id = cluster_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        # Calculate centroid
        centroid = np.mean(points, axis=0)
        marker.pose.position.x = centroid[0]
        marker.pose.position.y = centroid[1]
        marker.pose.position.z = 0

        # Calculate approximate size based on cluster spread
        points_array = np.array(points)
        max_dist = np.max(np.linalg.norm(points_array - centroid, axis=1))

        marker.scale.x = max_dist * 2
        marker.scale.y = max_dist * 2
        marker.scale.z = 0.1

        # Assign different colors to different clusters
        marker.color.r = (cluster_id * 0.3) % 1.0
        marker.color.g = (cluster_id * 0.5) % 1.0
        marker.color.b = (cluster_id * 0.7) % 1.0
        marker.color.a = 0.6

        return marker

    def scan_cb(self, msg):
        # Process raw LaserScan data
        min_angle = msg.angle_min
        max_angle = msg.angle_max
        start_angle = abs(max_angle) - abs(min_angle)
        angle_increment = msg.angle_increment

        # Filter valid ranges
        ranges = [i if (i > msg.range_min and i < msg.range_max) else -1
                  for i in msg.ranges]
        angles = [(start_angle + angle_increment*i)
                  for i in range(0, len(ranges))]

        # Convert to cartesian coordinates for clustering
        points = []
        for angle, range_val in zip(angles, ranges):
            if range_val != -1:  # Only include valid measurements
                points.append(self.polar_to_cartesian(angle, range_val))

        if not points:  # Skip if no valid points
            return

        # Perform DBSCAN clustering
        points_array = np.array(points)
        clustering = DBSCAN(
            eps=self.eps, min_samples=self.min_samples).fit(points_array)

        # Create markers for visualization
        marker_array = MarkerArray()
        unique_labels = set(clustering.labels_)

        for label in unique_labels:
            if label == -1:  # Skip noise points
                continue

            # Get points belonging to this cluster
            cluster_points = points_array[clustering.labels_ == label]

            # Create and add marker for this cluster
            marker = self.create_cluster_marker(cluster_points, label)
            marker_array.markers.append(marker)

        # Publish markers
        self.marker_pub.publish(marker_array)

        # Store processed data
        result = dict(zip(angles, ranges))
        self.lidar_ranges = result


if __name__ == '__main__':
    rospy.init_node('obstacle_clusterer')
    obstacle_clusterer = ObstacleClusterer()
    rospy.spin()
