from pathlib import Path

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.ros as ros

from ART1 import ART1

# =============================================================================


def AddConesFromFile(system, terrain, path_filename, cone_offset=(0, 0)):
    cone_offset_x, cone_offset_y = cone_offset

    red_cone_mesh_filename = chrono.GetChronoDataFile("sensor/cones/red_cone.obj")
    red_cone_mesh = chrono.ChTriangleMeshConnected()
    red_cone_mesh.LoadWavefrontMesh(red_cone_mesh_filename, False, True)

    green_cone_mesh_filename = chrono.GetChronoDataFile("sensor/cones/green_cone.obj")
    green_cone_mesh = chrono.ChTriangleMeshConnected()
    green_cone_mesh.LoadWavefrontMesh(green_cone_mesh_filename, False, True)

    red_cone_assets, green_cone_assets = [], []
    with open(chrono.GetChronoDataFile(path_filename)) as cone_file:
        while line := cone_file.readline():
            _, color, x, y = map(float, line.split("\t"))

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
            system.Add(cone_body)

    return red_cone_assets, green_cone_assets


def LabelAssets(assets, class_id):
    instance_id = 0
    for cone in assets:
        instance_id += 1
        for mat in cone.GetMaterials():
            mat.SetClassID(class_id)
            mat.SetInstanceID(instance_id)


# =============================================================================


def main():
    print("Running demo_ART_cone.py...")

    DATA_DIR = "/opt/chrono/share/chrono/data"
    chrono.SetChronoDataPath(f"{DATA_DIR}/")
    veh.SetDataPath(f"{DATA_DIR}/vehicle/")

    # Create the vehicle
    init_loc = chrono.ChVectorD(-2.2, 0.5, 0.5)
    init_rot = chrono.Q_from_AngZ(chrono.CH_C_PI / 2)
    vehicle = ART1(init_loc, init_rot)

    # Create the terrain
    terrain = veh.RigidTerrain(vehicle.GetSystem())

    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(
        patch_mat,
        chrono.CSYSNORM,
        7.0,
        7.0,
    )
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

    terrain.Initialize()

    # Create the sensor manager
    sensor_manager = sens.ChSensorManager(vehicle.GetSystem())
    sensor_manager.scene.AddPointLight(
        chrono.ChVectorF(0, 0, 100), chrono.ChColor(1, 1, 1), 5000
    )

    b = sens.Background()
    b.color_horizon = chrono.ChVectorF(0.6, 0.7, 0.8)
    b.color_zenith = chrono.ChVectorF(0.4, 0.5, 0.6)
    b.mode = sens.BackgroundMode_GRADIENT
    sensor_manager.scene.SetBackground(b)

    # Initialize the vehicle
    vehicle.Initialize(sensor_manager)

    # Create the ROS manager
    ros_manager = ros.ChROSManager()
    ros_manager.RegisterHandler(ros.ChROSClockHandler())
    ros_manager.RegisterHandler(
        ros.ChROSDriverInputsHandler(0, vehicle.driver, "~/input/driver_inputs")
    )
    ros_manager.RegisterHandler(
        ros.ChROSCameraHandler(
            vehicle.cam.GetUpdateRate(), vehicle.cam, "~/output/camera/data/image"
        )
    )
    ros_manager.RegisterHandler(
        ros.ChROSGPSHandler(
            vehicle.gps.GetUpdateRate(), vehicle.gps, "~/output/gps/data"
        )
    )
    ros_manager.RegisterHandler(
        ros.ChROSAccelerometerHandler(
            vehicle.acc.GetUpdateRate(), vehicle.acc, "~/output/acc/data"
        )
    )
    ros_manager.RegisterHandler(
        ros.ChROSGyroscopeHandler(
            vehicle.gyro.GetUpdateRate(), vehicle.gyro, "~/output/gyro/data"
        )
    )
    ros_manager.RegisterHandler(
        ros.ChROSMagnetometerHandler(
            vehicle.mag.GetUpdateRate(), vehicle.mag, "~/output/mag/data"
        )
    )
    ros_manager.Initialize()

    # Simulation Loop
    time = 0
    time_step = 1e-3
    time_end = 100

    realtime_timer = chrono.ChRealtimeStepTimer()
    while time < time_end:
        time = vehicle.GetSystem().GetChTime()

        driver_inputs = vehicle.driver.GetInputs()

        # Update modules (process inputs from other modules)
        vehicle.driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.veh.Synchronize(time, driver_inputs, terrain)

        # Advance simulation for one timestep for all modules
        vehicle.driver.Advance(time_step)
        terrain.Advance(time_step)
        vehicle.veh.Advance(time_step)

        # Update the sensor manager
        sensor_manager.Update()

        # Update the ROS manager
        if not ros_manager.Update(time, time_step):
            break

        # Spin in place for real time to catch up
        realtime_timer.Spin(time_step)


if __name__ == "__main__":
    main()
