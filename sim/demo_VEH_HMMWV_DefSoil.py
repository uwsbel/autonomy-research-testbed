# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2014 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Radu Serban
# =============================================================================
#
# Demonstration of vehicle over SCM deformable terrain
#
# The vehicle reference frame has Z up, X towards the front of the vehicle, and
# Y pointing to the left. All units SI.
#
# =============================================================================

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m
import numpy as np

# =============================================================================

def max(a, b):
    if a > b:
        return a
    else:
        return b

def min(a, b):
    if a < b:
        return a
    else:
        return b

class MyDriver (veh.ChDriver):
    def __init__(self, vehicle, delay, x, y):
        veh.ChDriver.__init__(self, vehicle)
        self.delay = delay
        self.vehh = vehicle

        self.dest_x = x
        self.dest_y = y

        self.steer = 0.0
        print("Destination is x: ", self.dest_x, " and y: ", self.dest_y)
    def Synchronize(self, time):
        eff_time = time - self.delay
        if (eff_time < 0):
            return

        # get vehicle position and orientation
#        print("x:", self.vehh.GetPos().x)
#        print("rot:", self.vehh.GetRot())
#        print("Eul:", self.vehh.GetRot().Q_to_Euler123())
#        print("NAS:",     self.vehh.GetRot().Q_to_NasaAngles())
        # Convert the quaternion to three angles (NASA angle set) heading, bank and attitude
        #print("Heading:", self.vehh.GetRot().Q_to_NasaAngles().z)
        heading = self.vehh.GetRot().Q_to_NasaAngles().z # should be attitude but looks like heading

        # calculate the correct orientation
        # dist_to_dest = np.sqrt( (self.vehh.GetPos().x ** 2) + (self.vehh.GetPos().y ** 2) )
        dist_to_dest = np.sqrt( (self.vehh.GetPos().x - self.dest_x) ** 2 + (self.vehh.GetPos().y - self.dest_y) ** 2 )

        angle_to_dest = m.acos( self.vehh.GetPos().y / dist_to_dest ) - (np.pi/2) # unsure why it starts at pi but whatever

        # incrementally adjust the vehicles direction to point towards the rock
        steer_delta = min(max(np.abs(angle_to_dest - heading) / 300.0, 0.000025 ), 0.0005) # 0.00035
        #steer_delta = np.abs(angle_to_dest - heading) / 100.0
        self.SetThrottle(0.9)
        if angle_to_dest > heading:
            self.steer += steer_delta
        else:
            self.steer -= steer_delta

        self.steer = sorted([-1, self.steer, 1])[1]
        self.SetSteering( self.steer )

        #print("Heading:", heading, "\t \t", angle_to_dest > heading, "\t", angle_to_dest, "\t \t", self.steer, "\t", int(dist_to_dest))
        print(steer_delta, "\t \t", self.steer, "\t", int(dist_to_dest))


def main():
    def AddObjRandom(pos, filename):
        mmesh = chrono.ChTriangleMeshConnected()
        mmesh.LoadWavefrontMesh(filename, False, True)
        mmesh.Transform(chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))
        for i in range(1):
            # Generate a random position
            x = -12.0 #(chrono.ChRandom() - .5) * cone_spread_x + cone_offset_x
            y = -4.0 #(chrono.ChRandom() - .5) * cone_spread_y + cone_offset_y
            z = terrain.GetHeight(chrono.ChVectorD(x, y, 1000)) # get the terrain z
            #pos = chrono.ChVectorD(x, y, z)
            #pos = chrono.ChVectorD(-12, -4, z)

            trimesh_shape = chrono.ChTriangleMeshShape()
            trimesh_shape.SetMesh(mmesh)
            trimesh_shape.SetName(filename)
            #trimesh_shape.SetStatic(True)
            trimesh_shape.SetScale(chrono.ChVectorD(1, 1, 1))

            mesh_body = chrono.ChBody()
            mesh_body.SetPos(pos)
            #mesh_body.SetRot(chrono.ChQuaternion(1, 0, 0, 0))
            mesh_body.AddVisualShape(trimesh_shape)
            mesh_body.SetBodyFixed(True)
            my_hmmwv.GetSystem().Add(mesh_body)


    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    #  Create the HMMWV vehicle, set parameters, and initialize
    my_hmmwv = veh.HMMWV_Full()
    my_hmmwv = veh.RCCar()
    my_hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
    my_hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(-27, -5, 0.3), chrono.ChQuaternionD(1, 0, 0, 0)))
    # uncommenting this immedietly crashes the vehicle
#    my_hmmwv.SetChassisCollisionType(veh.VisualizationType_PRIMITIVES)
    my_hmmwv.SetChassisFixed(False)
    #my_hmmwv.SetPowertrainType(veh.PowertrainModelType_SHAFTS)
    #my_hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    my_hmmwv.SetTireType(veh.TireModelType_RIGID)
    # if using TMEASY the tires don't cut into the ground
    #my_hmmwv.SetTireType(veh.TireModelType_TMEASY)
    my_hmmwv.Initialize()

    #my_hmmwv.SetChassisVisualizationType(veh.VisualizationType_NONE)
    my_hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    my_hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    my_hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    #my_hmmwv.SetWheelVisualizationType(veh.VisualizationType_NONE)
    my_hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    my_hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)
    # Create the RCCar vehicle, set parameters, and initialize
    #vehicle = veh.RCCar()
    #vehicle.SetContactMethod(contact_method)
    #vehicle.SetChassisCollisionType(chassis_collision_type)
    #vehicle.SetChassisFixed(False)
    #vehicle.SetTireType(tire_model)
    #vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
    #vehicle.SetTireStepSize(tire_step_size)
    #vehicle.Initialize()

    #vehicle.SetChassisVisualizationType(chassis_vis_type)
    #vehicle.SetSuspensionVisualizationType(suspension_vis_type)
    #vehicle.SetSteeringVisualizationType(steering_vis_type)
    #vehicle.SetWheelVisualizationType(wheel_vis_type)
    #vehicle.SetTireVisualizationType(tire_vis_type)


    # Create the (custom) driver
    driver = MyDriver(my_hmmwv.GetVehicle(), 0.4, final_dest_x, final_dest_y)
    driver.Initialize()

    # Create the SCM deformable terrain patch
    terrain = veh.SCMDeformableTerrain(my_hmmwv.GetSystem())
    terrain.SetSoilParameters(2e6,   # Bekker Kphi
                              0,     # Bekker Kc
                              1.1,   # Bekker n exponent
                              0,     # Mohr cohesive limit (Pa)
                              30,    # Mohr friction limit (degrees)
                              0.01,  # Janosi shear coefficient (m)
                              2e8,   # Elastic stiffness (Pa/m), before plastic yield
                              3e4    # Damping (Pa s/m), proportional to negative vertical speed (optional)
    )

    # Optionally, enable moving patch feature (single patch around vehicle chassis)
    terrain.AddMovingPatch(my_hmmwv.GetChassisBody(), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(5, 3, 1))

    # Set plot type for SCM (false color plotting)
    terrain.SetPlotType(veh.SCMDeformableTerrain.PLOT_SINKAGE, 0, 0.1);

    # Initialize the SCM terrain, specifying the initial mesh grid
    terrain.Initialize(terrainLength, terrainWidth, delta);

    # add rock objects
    
    rock_z_axis = terrain.GetHeight(chrono.ChVectorD(-12, -4, 1000))
    #AddObjPos( chrono.ChVectorD(-12, -4, rock_z_axis), chrono.GetChronoDataFile("robot/curiosity/rocks/rock1.obj") )
    #AddObjRandom( chrono.ChVectorD(-12, -4, 0), chrono.GetChronoDataFile("robot/curiosity/rocks/rock1.obj") )
    AddObjRandom( chrono.ChVectorD(final_dest_x, final_dest_y, 0), chrono.GetChronoDataFile("robot/curiosity/rocks/rock1.obj") )

    # Create the vehicle Irrlicht interface
    # Don't use irrlicht since it wants x server
    if USE_IRRLICHT:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle('HMMWV Deformable Soil Demo')
        vis.SetWindowSize(1280, 1024)
        vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
        vis.Initialize()
        vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
        vis.AddLightDirectional()
        vis.AddSkyBox()
        vis.AttachVehicle(my_hmmwv.GetVehicle())

        # Simulation loop
        while vis.Run() :
            time = my_hmmwv.GetSystem().GetChTime()

            # End simulation
            if (time >= sim_time_span):
                break

            # Draw scene
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

            # Get driver inputs
            driver_inputs = driver.GetInputs()


            # Update modules (process inputs from other modules)
            driver.Synchronize(time)
            terrain.Synchronize(time)
            my_hmmwv.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize("", driver_inputs)

            # Advance simulation for one timestep for all modules
            driver.Advance(step_size)
            terrain.Advance(step_size)
            my_hmmwv.Advance(step_size)
            vis.Advance(step_size)
        return 0
    # end if USE_IRRLICHT

    print("End sim", sim_time_span, " sec")
  

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# don't use irrlicht in dev container, use it in personal installation
USE_IRRLICHT = True
if not USE_IRRLICHT:
    print("Not using irrlicht")
    import pychrono.sensor as sens

# SCM patch dimensions
terrainHeight = 0
terrainLength = 64.08  # size in X direction meters
terrainWidth = 16.1    # size in Y direction meters

# final destination for vehicle
final_dest_x = -20
final_dest_y = -5

# max amount of time the simulation will run
sim_time_span = 20

# SCM grid spacing
delta = 0.05
#delta = 0.12
#delta = 0.01

# Simulation step sizes
step_size = 2e-3;       # default
tire_step_size = 1e-3;  # default
#step_size = 1e-3;
#tire_step_size = 5e-4;

main()
