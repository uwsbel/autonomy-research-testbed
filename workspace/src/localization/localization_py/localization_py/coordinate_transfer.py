#TANGENT PLANE MATH:

#x = r(cos(lat))(cos(lon))
#y = r(cos(lat)(sin(lon))
#z = r(sin(lat))
#Equation of Sphere: 0 = x^2+y^2+z^2-12,742,000 (in meters)
#F(x,y,z) = x^2+y^2+z^2-12,742,000 = 0
#G() = Gradient
#G()
import numpy as np
import math
class graph(object):
    def __init__(self):
        self.r = 6378100
    def transfer_relative(self, x, y, z):
        return x-self.orig[0], y-self.orig[1], z-self.orig[2]    

    def new_set_graph(self, lat, lon):
        #calculate the coordinates of where we are in 3 space
        self.calc_coordinates(lat, lon)
        #calculate the normal vector to our plane that we are projecting onto
        self.n_vec = np.array([-self.x, -self.y, -self.z])
        #the origin of where our 2D plane is
        self.orig = np.array([self.x, self.y, self.z])
        #calculating the projection of the i unit vector, along x axis

        v = np.array([(1-self.orig[0]), 0-self.orig[1], 0-self.orig[2]])
        d = v[0]*self.n_vec[0]+v[1]*self.n_vec[1]+v[2]*self.n_vec[2]
        #a point on the x_axis:
        x_axis_p = np.array(
            [d*self. n_vec[0]+v[0], d*self.n_vec[1]+v[1], d*self.n_vec[1]+v[1]])
        #our x axis vecotr
        self.x_axis_v = np.array(
            [x_axis_p[0]-self.orig[0], x_axis_p[1]-self.orig[1], x_axis_p[2]-self.orig[2]])
        #calculate the magnitude of this vector:
        self.mag_x = math.sqrt(
            self.x_axis_v[0]**2+self.x_axis_v[1]**2+self.x_axis_v[2]**2)
        #define the x-axis:
        #the Directional Ratios:
        #X_axis = np.array([proj[0]-self.orig[0], proj[1] - self.orig[1], proj[2]-self.orig[2]])
        #add the constant to the end of our DR, just easy storage...
        #np.append(X_axis,- (X_axis[0]*self.orig[0] +X_axis[1]*self.orig[1]+X_axis[2]*self.orig[2]))

        #self.i = np.array([])


    def set_graph(self,lat, lon):
        #calculate the coordinates of where we are in 3 space
        self.calc_coordinates(lat,lon)
        #calculate the normal vector to our plane that we are projecting onto
        self.n_vec = np.array([2*self.x,2*self.y,2*self.z])
        #the origin of where our 2D plane is
        self.orig = np.array([self.x,self.y,self.z])
        #calculating the projection of the i unit vector, along x axis
        
        v = np.array([(1-self.orig[0]),0-self.orig[1],0-self.orig[2]])
        d = v[0]*self.n_vec[0]+v[1]*self.n_vec[1]+v[2]*self.n_vec[2]
        #a point on the x_axis:
        x_axis_p = np.array([d*self.n_vec[0]+v[0], d*self.n_vec[1]+v[1], d*self.n_vec[1]+v[1] ])
        #our x axis vecotr
        self.x_axis_v = np.array(
            [x_axis_p[0]-self.orig[0], x_axis_p[1]-self.orig[1], x_axis_p[2]-self.orig[2]])
        #calculate the magnitude of this vector:
        self.mag_x = math.sqrt(
            self.x_axis_v[0]**2+self.x_axis_v[1]**2+self.x_axis_v[2]**2)
        #define the x-axis:
        #the Directional Ratios:
        #X_axis = np.array([proj[0]-self.orig[0], proj[1] - self.orig[1], proj[2]-self.orig[2]])
        #add the constant to the end of our DR, just easy storage...
        #np.append(X_axis,- (X_axis[0]*self.orig[0] +X_axis[1]*self.orig[1]+X_axis[2]*self.orig[2]))
        
        #self.i = np.array([])

    def calc_coordinates(self,lat,lon):
        #assume Earth is sphere centered at origin, N is Z.
        self.x = self.r*math.cos(math.radians(lat))*math.cos(math.radians(lon))
        self.y = self.r*math.cos(math.radians(lat))*math.sin(math.radians(lon))
        self.z = self.r*math.sin(math.radians(lat))

    def transfer_coordinates(self,lat,lon):
        self.calc_coordinates(lat,lon)
        #I think v has to be w/ out the orig
        #v = np.array([self.x,self.y,self.z])
        v= np.array([self.x-self.orig[0],self.y-self.orig[1],self.z-self.orig[2]])
        #TODO: need absolute value?
        d = ((v[0])*self.n_vec[0]+(v[1])*self.n_vec[1]+(v[2])*self.n_vec[2])
        #the projection of our point onto our plane
        proj = np.array([d*self.n_vec[0]+v[0], d*self.n_vec[1]+v[1], d*self.n_vec[2]+v[2]])
        #Now want to calculate the angle between our x axis and our current projected point.
        #first define projection vector from origin
        proj_v = np.array([proj[0]-self.orig[0], proj[1] -
                          self.orig[1], proj[2]-self.orig[2]])
        #calculate mag of proj. this is also our r in polar.
        
        #maybe just need to use proj, not proj_v??
        mag_proj = math.sqrt(proj[0]**2+proj[1]**2+proj[2]**2)
        theta = math.acos(
            (proj[0]*self.x_axis_v[0]+proj[1]*self.x_axis_v[1]+proj[2]*self.x_axis_v[2])/(self.mag_x*mag_proj))
        #the x and y, relative to the new plane:
        x = mag_proj*math.cos(theta)
        y = mag_proj*math.sin(theta)
        return v[0],self.x,self.y


#        mag_proj = math.sqrt(proj_v[0]**2+proj_v[1]**2+proj_v[2]**2)
#        theta = math.acos(
#            (proj_v[0]*self.x_axis_v[0]+proj_v[1]*self.x_axis_v[1]+proj_v[2]*self.x_axis_v[2])/(self.mag_x*mag_proj))
#        #the x and y, relative to the new plane:
#        x = mag_proj*math.cos(theta)
#        y = mag_proj*math.sin(theta)
#        return d,self.x,self.y
