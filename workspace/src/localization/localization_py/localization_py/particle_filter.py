import numpy as np
import math
import matplotlib.pyplot as plt
import random
from scipy.stats import wasserstein_distance
import yaml
from localization_py.dynamics import Dynamics

class ParticleFilter:
    """
    A basic Particle Filter implementation.
    
    This currently uses the 4DOF motion model. The Motion Model inputs are throttle and steering, and the observation model uses GPS and Magnetometer as sensors. States are cartesian coordinates x, y, heading theta, and velocity v. Further reference can be found at: https://sbel.wisc.edu/wp-content/uploads/sites/569/2023/07/TR-2023-05.pdf.

    Attributes:
        show_animation: Boolean for whether or not to show the particle animation.
        dt: The timestep for the filter.
        num_particles: The number of particles maintained by the filter.
        particles: The array of particles.
        particle_weights: The weight assigned to each particle for the final state produced.

        hpx: The array of x coordinate histories for each particle. Used for the animation.
        hpy: The array of y coordinate histories for each particle. Used for the animation.
        particle_distr_dist: The noise distribution for each particle, assuming that this particle is the true possition (relative to the measurement).
        particle_distr_head: The noise distribution for each particle, assuming that this particle is the true heading (relative to the measurement)
        dist_distr: The expected noise distribution for the position of the particles.
        head_distr: The expected noise distribution for the heading of the particles.
        timestep: the current timestep. Used for the animation.
    
    """

    def __init__(self, dt, dynamics):
        """Initialize the PF.

        Initialize each of the global variables for the PF. Load the dynamics parameters for the vehicle from an external YAML file.

        Args:
            dt: The timestep at which this filter will operate at. 
            dynamics: the parameters for the dynamics
        """
        self.dt = dt
        self.dyn = Dynamics(dt, dynamics)

        self.show_animation = False
        self.num_particles = 100
        self.particles = []
        self.particle_weights =[]
        self.hpx = []
        self.hpy = []
        self.particle_distr_dist = []
        self.particle_distr_head = []

        self.dist_distr = [abs(np.random.normal(0, 0.8)) for _ in range(100)]
        self.head_distr = [abs(np.random.normals(0,0.1)) for _ in range(100)]
        #counts, bins = np.histogram(points, bins = 30)

        for i in range(0,self.num_particles):
            self.hpx.append([])
            self.hpy.append([])
            self.particle_distr_dist.append([])
            self.particle_distr_head.append([])
            #self.particles.append(np.zeros((4,1)))
            self.particles.append(np.array([[np.random.normals(0,0.8)], [np.random.normals(0,0.8)], [0],[0]]))
            self.particle_weights.append(0.02)
        self.timestep = 0

    
    def update(self, u, obs):
        """The filter update function.

        This is the only function that gets called from outside of the PF object. It updates the entire model.

        Args:
            u: the 2-D control input applied to the vehicle, consisting of throttle and steering.
            obs: the 3-D observation from the sensors, consisting of cartesian coordinates x, y from GPS, and heading angle theta from the magnetometer measurement.

        Returns:
            A 4-D state for the vehicle, consisting of x, y, theta, and velocity v.

        """

        self.timestep = self.timestep+1
        #first, update the motion model
        for i in range(0, self.num_particles):
            myu = np.array([[u[0,0]+np.random.normal(0,0.05)], [u[1,0]+np.random.normal(0,0.05)]])
            self.particles[i] = self.dyn.motion_model(self.particles[i], myu)
        self.update_dist(obs)
        #then, reassign weights in relation to the observation
        self.assign_weights()

        state = np.zeros((4,1))
        for i in range(0,self.num_particles):
            for j in range(0,4):
                state[j,0] = state[j,0]+self.particles[i][j,0]*self.particle_weights[i]
        if(self.timestep%5 == 0):
            self.resample()
        #then, return final value
        return state
    
    def update_dist(self,obs):
        """Updates each particle error distribution from the observation.

        For each particle, compute the distance between the current state and the observation (position and heading). Add this to the associated distribution, and pop the oldest value.
        
        Args:
            obs: The 3-D observation vector.
        """

        for i in range(0, self.num_particles):
            distance = math.sqrt((self.particles[i][0,0]-obs[0,0])**2+(self.particles[i][1,0]-obs[1,0])**2)
            self.particle_distr_dist[i].append(distance)
            self.particle_distr_head[i].append(self.particles[i][2,0]-obs[2,0])
            if(len(self.particle_distr_dist[i])>10):
                self.particle_distr_dist[i].pop(0)
                self.particle_distr_head[i].pop(0)

    def resample(self):
        """Resample the particles for the next generation.

        Generates a new generation of particles. Computes a cummulative weight, and then generates num_particles random numbers. Then, whichever cummulative range the random number lies in is the particle that gets propogated. If a particle gets propogated more than once, some randomness is added to the state. 

        """

        cumulative_weights = [sum(self.particle_weights[:i+1]) for i in range(self.num_particles)]
        sampled_points = []
        for _ in range(self.num_particles):
            random_num = random.random()  # Generate a random number between 0 and 1

            # Find the interval where the random number falls
            for i, cum_weight in enumerate(cumulative_weights):
                if random_num < cum_weight:
                    sampled_points.append(i)
                    break
        already_seen = []
        new_particles = []
        new_pdd = []
        new_pdh = []
        new_hpx= []
        new_hpy = []
        for j in range(0,self.num_particles):
            i = sampled_points[j]
            new_pdd.append(self.particle_distr_dist[i].copy())
            new_pdh.append(self.particle_distr_head[i].copy())
            new_hpx.append(self.hpx[i].copy())
            new_hpy.append(self.hpy[i].copy())
            if i not in already_seen:
                already_seen.append(i)
                new_particles.append(self.particles[i].copy())
            else:
                new_particles.append(np.array([[self.particles[i][0,0]+np.random.normals(0,0.1)],[self.particles[i][1,0]+np.random.normals(0,0.1)],[self.particles[i][2,0]],[self.particles[i][3,0]]]))
        if(len(already_seen)<50):
            print("Particles Pruned: "+ str(50-len(already_seen)))
        self.particles = new_particles
        self.particle_distr_dist = new_pdd
        self.particle_distr_head = new_pdh
        self.hpx = new_hpx
        self.hpy = new_hpy
                



    def assign_weights(self):
        """Assigns weights to each particle.

        Assigns a weight to each particle based on the wasserstein distance between the current particle measurement probability distribution and the expected measurement probability distribution. 90% of weight is assigned to the position, and 10% weight is assigned to the heading. Normalizes the weights after each weight is assigned.

        """
        for i in range(0,self.num_particles):
            self.particle_weights[i] = 0.9/wasserstein_distance(self.particle_distr_dist[i], self.dist_distr)+0.1/wasserstein_distance(self.particle_distr_head[i], self.head_distr)
        self.normalizeWeights()

    def normalizeWeights(self):
        """Normalizes the weights.

        Normalizes the weights of all the particles, such that they sum to 1.
        """

        total = 0
        for i in range(0,self.num_particles):
            total = total+self.particle_weights[i]
        for i in range(0,self.num_particles):
            self.particle_weights[i] = self.particle_weights[i]/total
    
    
