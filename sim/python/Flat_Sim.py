#
# BSD 3-Clause License
#
# Copyright (c) 2022 University of Wisconsin - Madison
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.#

# General Imports
import numpy as np
import os

# Chrono Imports
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens

# // =============================================================================


class ChSystem_DataGeneratorFunctor(veh.ChExternalDriver_DataGeneratorFunctor):
    def __init__(self, id: str, system: chrono.ChSystem):
        super().__init__("ChSystem", id)

        self.system = system

    def Serialize(self, writer):
        writer.Key("time") << self.system.GetChTime()


class ChCameraSensor_DataGeneratorFunctor(veh.ChExternalDriver_DataGeneratorFunctor):
    def __init__(self, id: str, cam: sens.ChCameraSensor):
        super().__init__("ChCameraSensor", id)

        self.cam = cam

    def Serialize(self, writer):
        rgba8_buffer = self.cam.GetMostRecentRGBA8Buffer()
        if rgba8_buffer.HasData():
            rgba8_data = rgba8_buffer.GetRGBA8Data()
            rgba8_data = np.ascontiguousarray(
                np.flipud(rgba8_data)
            )  # Needs to be contigious
            shape = rgba8_data.shape
            writer.Key("width") << shape[1]
            writer.Key("height") << shape[0]
            writer.Key("size") << shape[2]
            writer.Key("encoding") << "rgba8"
            writer.Key("image").PointerAsString(
                rgba8_data.ctypes.data, int(np.prod(shape))
            )

    def HasData(self) -> bool:
        rgba8_buffer = self.cam.GetMostRecentRGBA8Buffer()
        return rgba8_buffer.HasData()


class ChVehicle_DataGeneratorFunctor(veh.ChExternalDriver_DataGeneratorFunctor):
    def __init__(self, id: str, vehicle: veh.ChVehicle):
        super().__init__("ChVehicle", id)

        self.vehicle = vehicle

    def Serialize(self, writer):
        body = self.vehicle.GetChassisBody()

        writer.Key("pos") << body.GetPos()
        writer.Key("rot") << body.GetRot()
        writer.Key("lin_vel") << body.GetPos_dt()
        writer.Key("ang_vel") << body.GetWvel_loc()
        writer.Key("lin_acc") << body.GetPos_dtdt()
        writer.Key("ang_acc") << body.GetWacc_loc()


class ChAccelerometerSensor_DataGeneratorFunctor(
    veh.ChExternalDriver_DataGeneratorFunctor
):
    def __init__(self, id: str, acc: sens.ChAccelerometerSensor):
        super().__init__("ChAccelerometerSensor", id)

        self.acc = acc

    def Serialize(self, writer):
        buffer = self.acc.GetMostRecentAccelBuffer()
        if buffer.HasData():
            data = buffer.GetAccelData()
            writer.Key("X") << data[0]
            writer.Key("Y") << data[1]
            writer.Key("Z") << data[2]

    def HasData(self) -> bool:
        buffer = self.acc.GetMostRecentAccelBuffer()
        return buffer.HasData()


class ChGyroscopeSensor_DataGeneratorFunctor(veh.ChExternalDriver_DataGeneratorFunctor):
    def __init__(self, id: str, gyro: sens.ChGyroscopeSensor):
        super().__init__("ChGyroscopeSensor", id)

        self.gyro = gyro

    def Serialize(self, writer):
        buffer = self.gyro.GetMostRecentGyroBuffer()
        if buffer.HasData():
            data = buffer.GetGyroData()
            writer.Key("roll") << data[0]
            writer.Key("pitch") << data[1]
            writer.Key("yaw") << data[2]

    def HasData(self) -> bool:
        buffer = self.gyro.GetMostRecentGyroBuffer()
        return buffer.HasData()


class ChMagnetometerSensor_DataGeneratorFunctor(
    veh.ChExternalDriver_DataGeneratorFunctor
):
    def __init__(self, id: str, mag: sens.ChMagnetometerSensor):
        super().__init__("ChMagnetometerSensor", id)

        self.mag = mag

    def Serialize(self, writer):
        buffer = self.mag.GetMostRecentMagnetBuffer()
        if buffer.HasData():
            data = buffer.GetMagnetData()
            writer.Key("X") << data[0]
            writer.Key("Y") << data[1]
            writer.Key("Z") << data[2]

    def HasData(self) -> bool:
        buffer = self.mag.GetMostRecentMagnetBuffer()
        return buffer.HasData()


class ChGPSSensor_DataGeneratorFunctor(veh.ChExternalDriver_DataGeneratorFunctor):
    def __init__(self, id: str, gps: sens.ChGPSSensor):
        super().__init__("ChGPSSensor", id)

        self.gps = gps

    def Serialize(self, writer):
        buffer = self.gps.GetMostRecentGPSBuffer()
        if buffer.HasData():
            data = buffer.GetGPSData().astype(str)
            writer.Key("latitude") << data[0]
            writer.Key("longitude") << data[1]
            writer.Key("altitude") << data[2]

    def HasData(self) -> bool:
        buffer = self.gps.GetMostRecentGPSBuffer()
        return buffer.HasData()


class ChDriverInputs_DataParserFunctor(veh.ChExternalDriver_DataParserFunctor):
    def __init__(self, driver: veh.ChDriver):
        super().__init__("ChDriverInputs")

        self.driver = driver

    def Deserialize(self, reader):
        steering = reader.GetFloat()
        throttle = reader.GetFloat()
        braking = reader.GetFloat()

        self.driver.SetThrottle(throttle)
        self.driver.SetSteering(steering)
        self.driver.SetBraking(braking)


# // =============================================================================


def main():
    def AddRandomCones(count, filename, class_id=1):
        mmesh = chrono.ChTriangleMeshConnected()
        mmesh.SetLoadWavefrontMesh(filename, False, True)
        mmesh.SetTransform(chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33(1))
        for i in range(count):
            # Generate a random position
            x = (chrono.ChRandom() - 0.5) * cone_spread_x + cone_offset_x
            y = (chrono.ChRandom() - 0.5) * cone_spread_y + cone_offset_y
            z = terrain.GetHeight(chrono.ChVectorD(x, y, 1000))  # get the terrain z
            pos = chrono.ChVectorD(x, y, z)

            trimesh_shape = chrono.ChTriangleMeshShape()
            trimesh_shape.SetMesh(mmesh)
            trimesh_shape.SetName(filename)
            trimesh_shape.SetStatic(True)
            trimesh_shape.SetScale(chrono.ChVectorD(1, 1, 1))

            mesh_body = chrono.ChBody()
            mesh_body.SetPos(pos)
            mesh_body.SetRot(chrono.ChQuaternion(1, 0, 0, 0))
            mesh_body.AddAsset(trimesh_shape)
            mesh_body.SetBodyFixed(True)
            vehicle.GetSystem().Add(mesh_body)

            if class_id == 1:
                red_cone_assets.append(trimesh_shape)
                cones.append(mesh_body)
            else:
                green_cone_assets.append(trimesh_shape)
                cones.append(mesh_body)

    def AddConesFromFile():
        green_cone_mesh = chrono.ChTriangleMeshConnected()
        green_cone_mesh.LoadWavefrontMesh(
            chrono.GetChronoDataFile("sensor/cones/green_cone.obj"), False, True
        )
        green_cone_mesh.Transform(chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))

        red_cone_mesh = chrono.ChTriangleMeshConnected()
        red_cone_mesh.LoadWavefrontMesh(
            chrono.GetChronoDataFile("sensor/cones/red_cone.obj"), False, True
        )
        red_cone_mesh.Transform(chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))

        with open(chrono.GetChronoDataFile(cone_path_file)) as cone_file:
            while True:
                line = cone_file.readline()
                if not line:
                    break
                id, color, x, y = map(float, line.split("\t"))

                pos_x = x + cone_offset_x
                pos_y = y + cone_offset_y
                pos_z = terrain.GetHeight(chrono.ChVectorD(pos_x, pos_y, 1000))

                pos = chrono.ChVectorD(pos_x, pos_y, pos_z)
                rot = chrono.ChQuaternionD(1, 0, 0, 0)

                cone_shape = chrono.ChTriangleMeshShape()
                if color == 0:
                    cone_shape.SetMesh(red_cone_mesh)
                    red_cone_assets.append(cone_shape)
                else:
                    cone_shape.SetMesh(green_cone_mesh)
                    green_cone_assets.append(cone_shape)

                cone_shape.SetStatic(True)

                cone_body = chrono.ChBody()
                cone_body.SetPos(pos)
                cone_body.SetRot(rot)
                cone_body.AddAsset(cone_shape)
                cone_body.SetBodyFixed(True)
                cones.append(cone_body)
                vehicle.GetSystem().Add(cone_body)

    def LabelConeAssets():
        cone_id = 0
        for cone in red_cone_assets:
            cone_id += 1
            for mat in cone.material_list:
                mat.SetClassID(1)
                mat.SetInstanceID(cone_id)

        cone_id = 0
        for cone in green_cone_assets:
            cone_id += 1
            for mat in cone.material_list:
                mat.SetClassID(2)
                mat.SetInstanceID(cone_id)

    # Create the RCCar vehicle, set parameters, and initialize
    vehicle = veh.RCCar()
    vehicle.SetContactMethod(contact_method)
    vehicle.SetChassisCollisionType(chassis_collision_type)
    vehicle.SetChassisFixed(False)
    vehicle.SetTireType(tire_model)
    vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
    vehicle.SetTireStepSize(tire_step_size)
    vehicle.Initialize()

    vehicle.SetChassisVisualizationType(chassis_vis_type)
    vehicle.SetSuspensionVisualizationType(suspension_vis_type)
    vehicle.SetSteeringVisualizationType(steering_vis_type)
    vehicle.SetWheelVisualizationType(wheel_vis_type)
    vehicle.SetTireVisualizationType(tire_vis_type)

    # Create the terrain
    terrain = veh.RigidTerrain(vehicle.GetSystem())

    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(
        0.01
    )  # -----check if mu, cr, and y is set similar to minfo

    # not right need to introduce point cloud -- not sure how to access this using the package_share_directory
    patch = terrain.AddPatch(
        patch_mat,
        chrono.ChVectorD(0, 0, 0),
        chrono.ChVectorD(0, 0, 1),
        terrainLength,
        terrainWidth,
    )
    patch.SetTexture(
        veh.GetDataFile("autonomy-toolkit/terrain/textures/tile4.jpg"), 200, 200
    )

    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    # patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, chrono.GetChronoDataFile("autonomy-toolkit/me3038/me3038_stereo_10cm_2.obj"))  #chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0))
    # patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, chrono.GetChronoDataFile("autonomy-toolkit/me3038/rm3038_pt_cloud_2.obj"))
    # patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    # patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # add in ME3038 room mesh
    # room_mmesh = chrono.ChTriangleMeshConnected()
    # room_mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("autonomy-toolkit/me3038/rm3038_pt_cloud.obj"), False, True)
    # room_mmesh.Transform(chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))

    # room_trimesh_shape = chrono.ChTriangleMeshShape()
    # room_trimesh_shape.SetMesh(room_mmesh)
    # room_trimesh_shape.SetName("ME3038")
    # room_trimesh_shape.SetStatic(True)
    # room_mesh_body = chrono.ChBody()
    # room_mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
    # room_mesh_body.AddAsset(room_trimesh_shape)
    # room_mesh_body.SetBodyFixed(True)

    # vehicle.GetSystem().Add(room_mesh_body)

    if cones_from_file:
        AddConesFromFile()
    # else:
    #    AddRandomCones(num_cones / 2, chrono.GetChronoDataFile("sensor/cones/green_cone.obj"), 2)
    #    AddRandomCones(num_cones / 2, chrono.GetChronoDataFile("sensor/cones/red_cone.obj"), 1)

    # === create sensors ===
    manager = sens.ChSensorManager(vehicle.GetSystem())
    manager.scene.AddPointLight(
        chrono.ChVectorF(0, 0, 100), chrono.ChVectorF(1, 1, 1), 5000
    )

    b = sens.Background()
    b.color_horizon = chrono.ChVectorF(0.6, 0.7, 0.8)
    b.color_zenith = chrono.ChVectorF(0.4, 0.5, 0.6)
    b.mode = sens.BackgroundMode_GRADIENT
    manager.scene.SetBackground(b)

    camera_pose = chrono.ChFrameD(
        chrono.ChVectorD(0.204, 0, 0.10018),
        chrono.Q_from_AngAxis(0.1, chrono.ChVectorD(0, 1, 0)),
    )
    width = 1280
    height = 720
    frame_rate = 30.0
    fov = 1.396  # 80 degree FOV camera)
    # fov = 1.59

    camera = sens.ChCameraSensor(
        vehicle.GetChassisBody(),  # body camera is attached to
        frame_rate,  # update rate in Hz
        camera_pose,  # offset pose
        width,  # image width
        height,  # image height
        fov,  # FOV
        2,
        sens.RADIAL,
    )  # super sample diameter
    camera.SetRadialLensParameters(chrono.ChVectorF(-0.369, 0.1257, -0.0194))
    camera.SetName("Camera Sensor")
    c_window = float(0)
    camera.SetCollectionWindow(c_window)
    # camera.SetLag(.5)

    lidar = sens.ChLidarSensor(
        vehicle.GetChassisBody(),  # body camera is attached to
        frame_rate,  # update rate in Hz
        camera_pose,  # offset pose
        width,  # image width
        height,  # image height
        fov,  # FOV
        2,
        sens.RADIAL,
    )

    if vis:
        camera.PushFilter(sens.ChFilterVisualize(640, 360))

    if save_sensor_data:
        camera.PushFilter(sens.ChFilterSave(sensor_data_dir + "cam1/"))

    camera.PushFilter(sens.ChFilterRGBA8Access())
    manager.AddSensor(camera)

    camera2 = sens.ChCameraSensor(
        patch.GetGroundBody(),  # body camera is attached to
        30,  # update rate in Hz
        chrono.ChFrameD(
            chrono.ChVectorD(-3.09, -2.89, 1.28),
            chrono.Q_from_Euler123(chrono.ChVectorD(0.0, 0.45, chrono.CH_C_PI_4 + 0.1)),
        ),  # offset pose   ------ not sure what CH_C_PI_2 is
        1920,  # image width
        1080,  # image height
        1.2,  # FOV
        2,
    )  # super sample diameter
    camera2.SetName("Camera Sensor 2")
    if vis:
        camera2.PushFilter(sens.ChFilterVisualize(640, 360))
    if save_sensor_data:
        camera2.PushFilter(sens.ChFilterSave(sensor_data_dir + "cam2/"))
    manager.AddSensor(camera2)

    noise_model = sens.ChNoiseNone()
    gps_noise_model = sens.ChNoiseNone()
    # gps_noise_model = sens.ChNoiseNormal(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0.6, 0.6, 0.6))
    imu_offset_pose = chrono.ChFrameD(
        chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(1, 0, 0))
    )
    gps_offset_pose = chrono.ChFrameD(
        chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(1, 0, 0))
    )
    gps_reference = chrono.ChVectorD(-89.400, 43.070, 260.0)

    # accelerometer
    acc = sens.ChAccelerometerSensor(
        vehicle.GetChassisBody(), 100, imu_offset_pose, noise_model
    )
    acc.SetName("IMU - Accelerometer")
    acc.PushFilter(sens.ChFilterAccelAccess())
    manager.AddSensor(acc)

    # gyroscope
    gyro = sens.ChGyroscopeSensor(
        vehicle.GetChassisBody(), 100, imu_offset_pose, noise_model
    )
    gyro.SetName("IMU - Gyroscope")
    gyro.PushFilter(sens.ChFilterGyroAccess())
    manager.AddSensor(gyro)

    # magnetometer
    mag = sens.ChMagnetometerSensor(
        vehicle.GetChassisBody(), 100, imu_offset_pose, noise_model, gps_reference
    )
    mag.SetName("IMU - Magnetometer")
    mag.PushFilter(sens.ChFilterMagnetAccess())
    manager.AddSensor(mag)

    # gps
    gps = sens.ChGPSSensor(
        vehicle.GetChassisBody(), 10, gps_offset_pose, gps_reference, gps_noise_model
    )
    gps.SetName("GPS")
    gps.PushFilter(sens.ChFilterGPSAccess())
    manager.AddSensor(gps)

    # Ground Truth
    groundTruth = sens.ChGPSSensor(
        vehicle.GetChassisBody(), 10, gps_offset_pose, gps_reference, gps_noise_model
    )
    groundTruth.SetName("groundTruth")
    groundTruth.PushFilter(sens.ChFilterGPSAccess())
    manager.AddSensor(groundTruth)

    # label the cones that were added to the scene with semantic information
    manager.ReconstructScenes()
    LabelConeAssets()
    manager.ReconstructScenes()

    driver = veh.ChExternalDriver(vehicle.GetVehicle(), 50000)

    system_generator = ChSystem_DataGeneratorFunctor(
        "~/output/time", vehicle.GetSystem()
    )
    driver.AddDataGenerator(system_generator)

    veh_generator = ChVehicle_DataGeneratorFunctor(
        "~/output/vehicle", vehicle.GetVehicle()
    )
    driver.AddDataGenerator(veh_generator, 10)

    cam_generator = ChCameraSensor_DataGeneratorFunctor(
        "~/output/camera/front_facing_camera", camera
    )
    driver.AddDataGenerator(cam_generator, frame_rate)

    acc_generator = ChAccelerometerSensor_DataGeneratorFunctor(
        "~/output/accelerometer/data", acc
    )
    driver.AddDataGenerator(acc_generator, 100)

    gyro_generator = ChGyroscopeSensor_DataGeneratorFunctor(
        "~/output/gyroscope/data", gyro
    )
    driver.AddDataGenerator(gyro_generator, 100)

    mag_generator = ChMagnetometerSensor_DataGeneratorFunctor(
        "~/output/magnetometer/data", mag
    )
    driver.AddDataGenerator(mag_generator, 100)

    gps_generator = ChGPSSensor_DataGeneratorFunctor("~/output/gps/data", gps)
    driver.AddDataGenerator(gps_generator, 10)

    groundTruth_generator = ChGPSSensor_DataGeneratorFunctor(
        "~/output/groundTruth/data", groundTruth
    )
    driver.AddDataGenerator(groundTruth_generator, 10)

    inputs_parser = ChDriverInputs_DataParserFunctor(driver)
    driver.AddDataParser(inputs_parser)

    # Create the vehicle Irrlicht interface
    if USE_IRRLICHT:
        app = veh.ChWheeledVehicleIrrApp(vehicle.GetVehicle(), "RC Car")
        app.AddTypicalLights()
        app.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
        app.SetChaseCamera(trackPoint, 15.0, 0.5)
        app.SetTimestep(step_size)
        app.AssetBindAll()
        app.AssetUpdateAll()

    # Simulation loop
    realtime_timer = chrono.ChRealtimeStepTimer()

    psteering = 0
    pthrottle = 0
    pbraking = 0

    throttle_time = 1.0
    steering_time = 1.0
    braking_time = 0.5
    delta_throttle = 1.0 * step_size / throttle_time
    delta_braking = 1.0 * step_size / braking_time
    delta_steering = 2.0 * step_size / steering_time

    while True:
        time = vehicle.GetSystem().GetChTime()

        # End simulation
        if (USE_IRRLICHT and not app.GetDevice().run()) or time >= t_end:
            break

        # Draw scene
        if USE_IRRLICHT:
            app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
            app.DrawAll()
            app.EndScene()

        # Get driver inputs
        driver_inputs = driver.GetInputs()

        # clamp driver inputs
        driver_inputs.m_steering

        pthrottle = np.clip(
            driver_inputs.m_throttle,
            pthrottle - delta_throttle,
            pthrottle + delta_throttle,
        )
        psteering = np.clip(
            driver_inputs.m_steering,
            psteering - delta_steering,
            psteering + delta_steering,
        )
        pbraking = np.clip(
            driver_inputs.m_braking, pbraking - delta_braking, pbraking + delta_braking
        )

        driver_inputs.m_throttle = pthrottle
        driver_inputs.m_steering = psteering
        driver_inputs.m_braking = pbraking

        # Update modules (process inputs from other modules)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        if USE_IRRLICHT:
            app.Synchronize("", driver_inputs)

        # Advance simulation for one timestep for all modules
        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)
        if USE_IRRLICHT:
            app.Advance(step_size)

        # Update sensor manager
        # Will render/save/filter automatically
        manager.Update()

        # Spin in place for real time to catch up
        realtime_timer.Spin(step_size)

    return 0


# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with:
data_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data", "")
chrono.SetChronoDataPath(data_folder)
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

vis = True
save_sensor_data = True
sensor_data_dir = "sensor_output/"

num_cones = 100
cones_from_file = False
# cone_path_file = "autonomy-toolkit/paths/cone_path_01.csv"
cone_path_file = "autonomy-toolkit/paths/cone_path_iros.csv"

cone_offset_x = 0
cone_offset_y = 0
cone_spread_x = 1.0
cone_spread_y = 1.0

throttle_scaling = (
    0.5 * abs(1400.0 - 1500.0) / (1980.0 - 1500.0)
)  # based on motor_driver.py limits for safety
braking_scaling = abs(1600.0 - 1500.0) / (
    1980.0 - 1500.0
)  # based on motor_driver.py limits for safety
steering_scaling = 1.0  # abs(1725.0 - 1500.0) / (1980.0 - 1500.0);  // based on motor_driver.py limits for safety

trackPoint = chrono.ChVectorD(0.0, 0.0, 1.75)

throttle = 0.0
braking = 0.0
steering = 0.0

red_cone_assets = list()
green_cone_assets = list()

cones = list()

# Initial vehicle location
init_loc_x = -2.2
init_loc_y = 0.5
init_angle_z = 1.57

initLoc = chrono.ChVectorD(init_loc_x, init_loc_y, 0.5)
initRot = chrono.Q_from_AngZ(init_angle_z)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
chassis_vis_type = veh.VisualizationType_PRIMITIVES
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain -- not sure about this
terrainHeight = -0.2339  # terrain height (FLAT terrain only)
terrainLength = 7.0  # size in X direction
terrainWidth = 7.0  # size in Y direction

# Contact method
contact_method = chrono.ChContactMethod_NSC
# contact_vis = False

# Flag to activate irrlicht
USE_IRRLICHT = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Simulation end time
t_end = 1000

main()
