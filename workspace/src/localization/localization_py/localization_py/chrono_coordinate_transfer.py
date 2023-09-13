import numpy as np
import math
class graph:
    '''
    This class is a copy of the CHRONO LTP gps2cartesian coordinate transfer.
    Additional features for rotating the LTP, and setting the rotation are added.

    Chrono documentation: https://api.projectchrono.org/namespacechrono_1_1sensor.html#a69c276ee1766b50936ed940d39a2555d
    '''
    def __init__(self):
        self.r = self.r = 6378100

    def set_graph(self, lat, lon, alt):
        self.lat = lat
        self.lon = lon
        self.alt = alt

    def gps2cartesian(self,lat,lon,alt):
        x = ((lon - self.lon) * math.pi / 180.0) * (self.r * math.cos(lat * math.pi / 180.0))
        y = ((lat - self.lat) * math.pi / 180.0) * self.r
        z = alt - self.alt
        return x,y,z

    def set_rotation(self, D):
        self.R = np.array([
            [math.cos(D), -math.sin(D)],
            [math.sin(D), math.cos(D)]
        ])
        self.R = np.linalg.inv(self.R)
    
    def rotate(self, x, y, z):
        position = np.array([
            [x],
            [y]
        ])
        position = self.R @ position

        return position[0][0], position[1][0],z
