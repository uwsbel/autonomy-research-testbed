import numpy as np
import math
from scipy.spatial.transform import Rotation
class graph(object):
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
    def set_new_rotation(self, quat):
        self.R = Rotation.from_quat(quat).as_matrix()
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
