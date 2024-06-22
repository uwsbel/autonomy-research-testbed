from pathlib import Path
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.ros as ros
from ART1 import ART1
from utils import AddConesFromFile, LabelAssets

def create_vehicle(init_loc, init_rot):
    return ART1(init_loc, init_rot)

def setup_sensor_manager(system):
    sensor_manager = sens.ChSensorManager(system)
    sensor_manager.scene.AddPointLight(
        chrono.ChVector3f(0, 0, 100), chrono.ChColor(1, 1, 1), 5000
    )

    b = sens.Background()
    b.color_horizon = chrono.ChVector3f(0.6, 0.7, 0.8)
    b.color_zenith = chrono.ChVector3f(0.4, 0.5, 0.6)
    b.mode = sens.BackgroundMode_GRADIENT
    sensor_manager.scene.SetBackground(b)

    return sensor_manager

def setup_terrain(system):
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()
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
    return terrain

def main(args):
    print("Running demo_ART_cone.py...")

    DATA_DIR = "/opt/chrono/share/chrono/data"
    chrono.SetChronoDataPath(f"{DATA_DIR}/")
    veh.SetDataPath(f"{DATA_DIR}/vehicle/")

    # Initialize the vehicle locations and rotations
    init_locs = [chrono.ChVector3d(-2.2, 0.5, 0.2), chrono.ChVector3d(2.2, 0.5, 0.2)]
    init_rot = chrono.ChQuaterniond()
    init_rot.SetFromAngleZ(3.14159 / 2)

    # Create vehicles and their sensor managers
    vehicles = [create_vehicle(loc, init_rot) for loc in init_locs]
    sensor_managers = [setup_sensor_manager(vehicle.GetSystem()) for vehicle in vehicles]

    # Initialize terrain
    terrain = setup_terrain(vehicles[0].GetSystem())

    # Initialize vehicles with their sensor managers
    for vehicle, sensor_manager in zip(vehicles, sensor_managers):
        vehicle.Initialize(sensor_manager)
        sensor_manager.ReconstructScenes()

    # Add an Irrlicht visualization, if desired
    if args.irrlicht:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("ART1")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.0), 1.0, 0.1)
        vis.Initialize()
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
        vis.AddLightDirectional()
        vis.AddSkyBox()
        for vehicle in vehicles:
            vis.AttachVehicle(vehicle.GetVehicle())

    # Simulation loop
    time = 0
    time_step = 1e-3
    time_end = 100
    realtime_timer = chrono.ChRealtimeStepTimer()

    while time < time_end:
        if args.irrlicht:
            if not vis.Run():
                break
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        time = vehicles[0].GetSystem().GetChTime()

        for vehicle in vehicles:
            driver_inputs = vehicle.driver.GetInputs()
            vehicle.driver.Synchronize(time)
            vehicle.Synchronize(time, driver_inputs, terrain)
            vehicle.driver.Advance(time_step)
            vehicle.Advance(time_step)

        terrain.Synchronize(time)
        terrain.Advance(time_step)

        for sensor_manager in sensor_managers:
            sensor_manager.Update()

        if args.realtime:
            realtime_timer.Spin(time_step)

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser("Demo of ART1 with cones")
    parser.add_argument("--track", action="store_true", help="Enable tracking camera")
    parser.add_argument(
        "--irr",
        "--irrlicht",
        dest="irrlicht",
        action="store_true",
        help="Enable Irrlicht",
    )
    parser.add_argument("--realtime", action="store_true", help="Enable real time")
    args = parser.parse_args()
    main(args)
