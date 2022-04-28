import numpy as np
import math
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
