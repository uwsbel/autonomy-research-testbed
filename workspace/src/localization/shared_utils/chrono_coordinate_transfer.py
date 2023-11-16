import numpy as np
import math


class Graph:
    """A copy of the CHRONO LTP gps2cartesian coordinate transfer.

    Generates cartesian coordinates from gps coordinates relative to a Local Tangent Plane. Additional features for rotating the LTP are added on top of the standard CHRONO methods.

    Attributes:
        r: The radius of the Earth.
        lat: The Latitude for the origin of the LTP.
        lon: The Longitude for the origin of the LTP.
        alt: The Altitude for the origin of the LTP.
        R: The 2D rotation matrix for our LTP. It must still be tangent to the Earth's surface so we only have 2 degrees of freedom for rotation.

    Chrono documentation: https://api.projectchrono.org/namespacechrono_1_1sensor.html#a69c276ee1766b50936ed940d39a2555d
    """

    def __init__(self):
        """Initialize the graph instance."""
        self.r = 6378100

    def set_graph(self, lat, lon, alt):
        """Sets the origin of the graph.

        Sets the origin of the LTP based off of given GPS inputs.

        Args:
            lat: The Latitude for the origin of the LTP.
            lon: The Longitude for the origin of the LTP.
            alt: The Altitude for the origin of the LTP.
        """
        self.lat = lat
        self.lon = lon
        self.alt = alt

    def gps2cartesian(self, lat, lon, alt):
        """Translates GPS coordinates to our LTP frame.

        Projects the GPS coordinates onto the 2D LTP, and then returns the coordinates relative to the unrotated LTP.

        Args:
            lat: The Latitude for the point we want to translate.
            lon: The Longitude for the point we want to translate.
            alt: The Altitude for the point we want to translate.
        """
        x = ((lon - self.lon) * math.pi / 180.0) * (
            self.r * math.cos(lat * math.pi / 180.0)
        )
        y = ((lat - self.lat) * math.pi / 180.0) * self.r
        z = alt - self.alt
        return x, y, z

    def set_rotation(self, D):
        """Sets the 2D rotation matrix for the LTP.

        Args:
            D: The offset angle we want for the x-axis of our LTP.
        """
        self.R = np.array([[math.cos(D), -math.sin(D)], [math.sin(D), math.cos(D)]])
        self.R = np.linalg.inv(self.R)

    def rotate(self, x, y, z):
        """Rotates a given point relative to the rotation matrix.

        Args:
            x: the x-coordinate we would like to rotate.
            y: the y-coordinate we would like to roatate.
            z: the z-coordinate.
        """
        position = np.array([[x], [y]])
        position = self.R @ position

        return position[0][0], position[1][0], z
