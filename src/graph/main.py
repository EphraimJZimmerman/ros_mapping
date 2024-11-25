import math

"""

TODO:
Pre-derive the graph structure 

"""


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
        a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
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

    def get_edge_direction(self, current_lat, current_lon, current_yaw) -> tuple:
        """Given the robot's current position and yaw, return the closest node and direction to the next node."""

        closest_node = self.find_closest_node(current_lat, current_lon)
        # Find the next node in the closest node's edge list

        if self.edges[closest_node]:
            next_node = self.edges[closest_node][0]
            next_lat, next_lon = next_node
            # Calculate the bearing (direction) from current node to the next node
            bearing = self._calculate_bearing(current_lat, current_lon, next_lat, next_lon)

            return closest_node, next_node, (bearing - current_yaw) % 360
        else:
            print(f"Error: No edges connected to node {closest_node}")
            return None

    def _calculate_bearing(self, lat1, lon1, lat2, lon2) -> float:
        """Calculate the bearing from point (lat1, lon1) to point (lat2, lon2)."""

        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_lambda = math.radians(lon2 - lon1)
        x = math.sin(delta_lambda) * math.cos(phi2)
        y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)
        initial_bearing = math.atan2(x, y)

        # Convert to degrees and normalize to 0-360 range
        initial_bearing = math.degrees(initial_bearing)
        return (initial_bearing + 360) % 360


# Example Usage
graph = Graph()
graph.add_node(0, -0)
graph.add_node(0, -0)
graph.add_edge((0, -0), (0, -0))

# Robot's current location and yaw (in degrees)
robot_lat = 0
robot_lon = -0
robot_yaw = 90  # Facing east

closest_node, next_node, direction = graph.get_edge_direction(robot_lat, robot_lon, robot_yaw)
print(closest_node)
print(next_node)
print(direction)
