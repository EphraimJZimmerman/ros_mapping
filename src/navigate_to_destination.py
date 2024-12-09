#! /usr/bin/python3
import rospy
import math
from collections import deque
from std_msgs.msg import Float64
from sensor_msgs.msg import MagneticField, NavSatFix

# Global variables to store the robot's yaw and GPS coordinates
current_yaw = 0  # Default yaw
current_lat = None
current_lon = None


def gps_callback(msg):
    global current_lat, current_lon
    current_lat = msg.latitude
    current_lon = msg.longitude
    rospy.loginfo(
        f"Updated GPS: Latitude = {current_lat}, Longitude = {current_lon}")

# Magnetic field callback to update yaw


def magnetic_callback(msg):
    global current_yaw
    Bx = msg.magnetic_field.x
    By = msg.magnetic_field.y
    yaw_radians = math.atan2(By, Bx)
    current_yaw = (math.degrees(yaw_radians) + 360) % 360
    # rospy.loginfo(f"Updated Yaw: {current_yaw:.2f} degrees")


class Graph:
    def __init__(self):
        # Graph will hold nodes as lat/lon tuples and edges as pairs of nodes
        self.nodes = []  # List of nodes (lat, lon)
        self.edges = {}  # Dictionary mapping node to list of connected nodes

    def add_node(self, lat, lon) -> None:
        self.nodes.append((lat, lon))
        self.edges[(lat, lon)] = []

    def add_edge(self, node1, node2) -> None:
        """Adds an undirected edge between two nodes"""
        if node1 in self.edges and node2 in self.edges:
            self.edges[node1].append(node2)
            self.edges[node2].append(node1)
        else:
            print("We don't have that node!")

    def _haversine_distance(self, lat1, lon1, lat2, lon2):
        """Calculate the Haversine distance between two lat/lon points in meters.
        Copied directly from a StackOverflow article on this question, thanks!

        https://en.wikipedia.org/wiki/Haversine_formula
        """

        R = 6371000  # Radius of Earth in meters
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * \
            math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c
        return distance

    def find_closest_node(self, lat, lon):
        """Find the closest node to a given lat/lon point."""
        closest_node = None
        min_distance = float('inf')

        for node_lat, node_lon in self.nodes:
            distance = self._haversine_distance(lat, lon, node_lat, node_lon)
            if distance < min_distance:
                closest_node = (node_lat, node_lon)
                min_distance = distance

        return closest_node

    def get_edge_direction(self, current_lat, current_lon, current_yaw, target_lat, target_lon) -> tuple:
        """
        Given the robot's current position (lat, lon) and yaw, this method calculates the bearing
        to a target node and the necessary rotation to align the robot's yaw with that bearing.
        """
        # Calculate the bearing from current position to the target node
        bearing_to_target = self._calculate_bearing(
            current_lat, current_lon, target_lat, target_lon)

        # Calculate how much the robot needs to turn from its current yaw to the bearing
        # Normalize the difference to be between -180 and 180 degrees
        turn_angle = (bearing_to_target - current_yaw + 180) % 360 - 180

        return bearing_to_target, turn_angle

    def _calculate_bearing(self, lat1, lon1, lat2, lon2) -> float:
        """Calculate the bearing from point (lat1, lon1) to point (lat2, lon2)."""

        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_lambda = math.radians(lon2 - lon1)
        x = math.sin(delta_lambda) * math.cos(phi2)
        y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * \
            math.cos(phi2) * math.cos(delta_lambda)
        initial_bearing = math.atan2(x, y)

        # Convert to degrees and normalize to 0-360 range
        initial_bearing = math.degrees(initial_bearing)
        return (initial_bearing + 360) % 360

    def bfs(self, start_node, end_node):
        """Find the shortest path from start_node to end_node using BFS."""
        # Queue to keep track of nodes to visit, each element is (current_node, path_taken)
        queue = deque([(start_node, [start_node])])
        visited = set()  # Keep track of visited nodes to avoid revisiting

        while queue:
            current_node, path = queue.popleft()

            # If we reach the target node, return the path
            if current_node == end_node:
                return path

            # Mark current node as visited
            visited.add(current_node)

            # Explore neighbors (connected nodes)
            for neighbor in self.edges.get(current_node, []):
                if neighbor not in visited:
                    # Add neighbor and the updated path to the queue
                    queue.append((neighbor, path + [neighbor]))

        return None  # Return None if there's no path from start to end


def main():
    # Initialize node
    rospy.init_node('robot_navigation', anonymous=True)

    # ROS publishers for bearing and turn angle
    bearing_pub = rospy.Publisher('/robot/bearing', Float64, queue_size=10)
    turn_angle_pub = rospy.Publisher(
        '/robot/turn_angle', Float64, queue_size=10)
    rospy.Subscriber("/imu/mag_corrected", MagneticField, magnetic_callback)

    rospy.Subscriber("/imu/mag_corrected", MagneticField, magnetic_callback)
    rospy.Subscriber("/gps", NavSatFix, gps_callback)

    rate = rospy.Rate(1)  # Publish at 1 Hz

    # Define locations for nodes a to j
    locations = {
        "a": (42.365901, -71.259747),
        "b": (42.365823, -71.259422),
        "c": (42.366028, -71.258998),
        "f": (42.366346, -71.259256),
        "d": (42.366097, -71.259606),
        "e": (42.366149, -71.259970),
        "g": (42.366016, -71.259441),
        "h": (42.366278, -71.259214),
        "i": (42.366260, -71.259564),
        "j": (42.365874, -71.259272)
    }

    graph = Graph()
    for loc in locations.values():
        graph.add_node(loc[0], loc[1])

    graph.add_edge(locations["a"], locations["b"])
    graph.add_edge(locations["b"], locations["j"])
    graph.add_edge(locations["j"], locations["c"])
    graph.add_edge(locations["c"], locations["h"])
    graph.add_edge(locations["h"], locations["f"])
    graph.add_edge(locations["h"], locations["g"])
    graph.add_edge(locations["g"], locations["a"])
    graph.add_edge(locations["f"], locations["i"])
    graph.add_edge(locations["i"], locations["d"])
    graph.add_edge(locations["d"], locations["a"])
    graph.add_edge(locations["f"], locations["i"])
    graph.add_edge(locations["i"], locations["e"])
    graph.add_edge(locations["e"], locations["a"])

    # Starting point and target node, currently static
    start_node = locations["c"]  # Robot's current location (node "a")
    end_node = locations["a"]  # Destination node ("c")

    if current_lat is None or current_lon is None:
        rospy.logwarn("Waiting for GPS fix...")
        rospy.wait_for_message("/gps", NavSatFix)

    # Localize the robot to the closest node
    closest_node = graph.find_closest_node(start_node[0], start_node[1])

    # Find the shortest path using BFS
    path = graph.bfs(closest_node, end_node)

    if path:
        print("Path found")
        for i in range(len(path) - 1):

            current_node = path[i]
            next_node = path[i + 1]

            current_lat, current_lon = current_node
            next_lat, next_lon = next_node

            # Wait until the robot reaches the current node (within a small threshold distance)
            threshold_distance = 1.0  # 1 meter threshold for arriving at a node
            print(
                f"Distance to next node is {graph._haversine_distance(current_lat, current_lon, current_lat, current_lon)}")
            while graph._haversine_distance(current_lat, current_lon, current_lat, current_lon) > threshold_distance:
                print("waiting to arrive")
                # Wait for the robot to reach the current node
                rospy.sleep(0.1)

            # Assuming robot's current yaw is updated from the magnetic field sensor
            robot_yaw = current_yaw

            # Calculate bearing and turn angle to the next node
            bearing_to_target, turn_angle = graph.get_edge_direction(
                current_lat, current_lon, robot_yaw, next_lat, next_lon)

            # Publish bearing and turn angle for the next node
            rospy.loginfo(
                f"Publishing bearing: {bearing_to_target} and turn angle: {turn_angle}")
            bearing_pub.publish(bearing_to_target)
            turn_angle_pub.publish(turn_angle)

            rate.sleep()  # Sleep to maintain the desired publishing rate


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
