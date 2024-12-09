import math

"""
decimal
places   degrees          distance
-------  -------          --------
0        1                111  km
1        0.1              11.1 km
2        0.01             1.11 km
3        0.001            111  m
4        0.0001           11.1 m
5        0.00001          1.11 m
6        0.000001         11.1 cm
7        0.0000001        1.11 cm
8        0.00000001       1.11 mm

"""

# Approximate radius of the Earth in meters
LAT_LON_TO_METERS = 111320

#https://en.wikipedia.org/wiki/Small-angle_approximation

def is_within_distance(lat1, lon1, lat2, lon2, max_distance=3):

    dlat = lat2 - lat1 # Convert latitude and longitude differences to meters
    dlon = lon2 - lon1

    # Approximate the distance using euclidian distance
    distance = math.sqrt(
        (dlat * LAT_LON_TO_METERS) ** 2 + (dlon * LAT_LON_TO_METERS * math.cos(math.radians(lat1))) ** 2)

    # Check if the distance is within the max_distance
    return distance <= max_distance


# Example usage:
point1 = (42.366035, -71.259047)
point2 = (42.366054, -71.259029)  # A nearby point

if is_within_distance(point1[0], point1[1], point2[0], point2[1]):
    print("The points are within 3 meters of each other.")
else:
    print("The points are more than 3 meters apart.")
