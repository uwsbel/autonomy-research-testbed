from pathlib import Path
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.ros as ros
from ART1 import ART1
from utils import AddConesFromFile, LabelAssets

def create_vehicle_and_ros_manager(init_loc, init_rot, data_path, args, namespace):
    # Create the vehicle
    vehicle = ART1(init_loc, init_rot)

    # Create the terrain
    terrain = veh.RigidTerrain(vehicle.GetSystem())
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 7.0, 7.0)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # Add cones
    red_cones, green_cones = AddConesFromFile(vehicle.GetSystem(), terrain, data_path / "cone_paths" / "cone_path_iros.csv")

    # Create the sensor manager
    sensor_manager = sens.ChSensorManager(vehicle.GetSystem())
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(0, 0, 100), chrono.ChColor(1, 1, 1), 5000)
    b = sens.Background()
    b.color_horizon = chrono.ChVector3f(0.6, 0.7, 0.8)
    b.color_zenith = chrono.ChVector3f(0.4, 0.5, 0.6)
    b.mode = sens.BackgroundMode_GRADIENT
    sensor_manager.scene.SetBackground(b)

    # Initialize the vehicle
    vehicle.Initialize(sensor_manager)

    # Add an irrlicht visualization, if desired
    vis = None
    if args.irrlicht:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle(f"ART1 - {namespace}")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.0), 1.0, 0.1)
        vis.Initialize()
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
        vis.AddLightDirectional()
        vis.AddSkyBox()
        vis.AttachVehicle(vehicle.GetVehicle())

    # Add a vehicle camera follower, if desired
    if args.track:
        q = chrono.ChQuaterniond()
        q.SetFromEuler(chrono.ChVector3d(0.0, 0.45, (3.14159 / 4) + 0.1))
        offset_pose = chrono.ChFramed(chrono.ChVector3d(-3.09, -2.89, 1.28), q)
        tracking_cam = sens.ChCameraSensor(
            vehicle.GetChassisBody(),
            30,
            offset_pose,
            1280,
            720,
            3.14159 / 3,
            2,
        )
        tracking_cam.PushFilter(sens.ChFilterVisualize(1280, 720, "Tracking Camera"))
        sensor_manager.AddSensor(tracking_cam)

    sensor_manager.ReconstructScenes()
    LabelAssets(red_cones, 1)
    LabelAssets(green_cones, 2)
    sensor_manager.ReconstructScenes()

    # Create the ROS manager
    ros_manager = ros.ChROSManager(namespace)
    ros_manager.RegisterHandler(ros.ChROSClockHandler())
    ros_manager.RegisterHandler(ros.ChROSDriverInputsHandler(0, vehicle.driver, f"/{namespace}/control/vehicle_inputs"))
    ros_manager.RegisterHandler(ros.ChROSCameraHandler(vehicle.cam.GetUpdateRate(), vehicle.cam, f"/{namespace}/sensing/front_facing_camera/raw"))
    ros_manager.RegisterHandler(ros.ChROSGPSHandler(vehicle.gps.GetUpdateRate(), vehicle.gps, f"/{namespace}/sensing/gps/data"))
    ros_manager.RegisterHandler(ros.ChROSAccelerometerHandler(vehicle.acc.GetUpdateRate(), vehicle.acc, f"/{namespace}/sensing/accelerometer/data"))
    ros_manager.RegisterHandler(ros.ChROSGyroscopeHandler(vehicle.gyro.GetUpdateRate(), vehicle.gyro, f"/{namespace}/sensor/gyroscope/data"))
    ros_manager.RegisterHandler(ros.ChROSMagnetometerHandler(vehicle.mag.GetUpdateRate(), vehicle.mag, f"/{namespace}/sensing/magnetometer/data"))

    ros_manager.Initialize()

    return vehicle, terrain, sensor_manager, ros_manager, vis

def main(args):
    print("Running demo_ART_cone.py...")
    DATA_DIR = "/opt/chrono/share/chrono/data"
    chrono.SetChronoDataPath(f"{DATA_DIR}/")
    veh.SetDataPath(f"{DATA_DIR}/vehicle/")
    data_path = Path(__file__).parent.parent / "data"

    # Initialize two vehicles with different namespaces
    vehicle1, terrain1, sensor_manager1, ros_manager1, vis1 = create_vehicle_and_ros_manager(
        chrono.ChVector3d(-2.2, 0.5, 0.2),
        chrono.ChQuaterniond(),
        data_path,
        args,
        "vehicle1"
    )

    vehicle2, terrain2, sensor_manager2, ros_manager2, vis2 = create_vehicle_and_ros_manager(
        chrono.ChVector3d(2.2, 0.5, 0.2),
        chrono.ChQuaterniond(),
        data_path,
        args,
        "vehicle2"
    )

    # Simulation Loop
    time = 0
    time_step = 1e-3
    time_end = 100

    realtime_timer = chrono.ChRealtimeStepTimer()
    while time < time_end:
        if args.irrlicht:
            if not vis1.Run() or not vis2.Run():
                break
            vis1.BeginScene()
            vis1.Render()
            vis1.EndScene()
            vis2.BeginScene()
            vis2.Render()
            vis2.EndScene()

        time = vehicle1.GetSystem().GetChTime()

        driver_inputs1 = vehicle1.driver.GetInputs()
        driver_inputs2 = vehicle2.driver.GetInputs()

        # Update modules (process inputs from other modules)
        vehicle1.driver.Synchronize(time)
        terrain1.Synchronize(time)
        vehicle1.Synchronize(time, driver_inputs1, terrain1)
        vehicle2.driver.Synchronize(time)
        terrain2.Synchronize(time)
        vehicle2.Synchronize(time, driver_inputs2, terrain2)

        # Advance simulation for one timestep for all modules
        vehicle1.driver.Advance(time_step)
        terrain1.Advance(time_step)
        vehicle1.Advance(time_step)
        vehicle2.driver.Advance(time_step)
        terrain2.Advance(time_step)
        vehicle2.Advance(time_step)

        # Update the sensor managers
        sensor_manager1.Update()
        sensor_manager2.Update()

        # Update the ROS managers
        if not ros_manager1.Update(time, time_step) or not ros_manager2.Update(time, time_step):
            break

        # Spin in place for real time to catch up
        if args.realtime:
            realtime_timer.Spin(time_step)

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser("Demo of ART1 with cones")
    parser.add_argument("--track", action="store_true", help="Enable tracking camera")
    parser.add_argument("--irr", "--irrlicht", dest="irrlicht", action="store_true", help="Enable Irrlicht")
    parser.add_argument("--realtime", action="store_true", help="Enable real time")

    args = parser.parse_args()
    main(args)
