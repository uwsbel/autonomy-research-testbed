import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.geometry as geo
import pychrono.sensor as sensor
import math




create_semantic_maps = False
save_sensor_data = False
sensor_data_dir = "sensor_output/"

num_cones = 100
cones_from_file = True
cone_path_file = "data/paths/cone_paths_0.csv"

cone_offset_x = 0
cone_offset_y = 0
cone_spread_x = 1.0
cone_spread_y = 1.0
init_loc_x = 0
init_loc_y = 0
init_angle_z = 0

throttle_scaling = .5 * abs(1400.0 - 1500.0) / (1980.0 - 1500.0) # based on motor_driver.py limits for safety
braking_scaling = abs(1600.0 - 1500.0) / (1980.0 - 1500.0)  # based on motor_driver.py limits for safety
steering_scaling = 1.0  # abs(1725.0 - 1500.0) / (1980.0 - 1500.0);  // based on motor_driver.py limits for safety


pub_frequency = 10.0

sim_step = 0
pub_count = 0

# control inputs   - not sure if I need this
throttle = 0.0
braking = 0.0
steering = 0.0

# simulation variables
# std::shared_ptr<RCCar> vehicle;
# std::shared_ptr<ChDriver> driver;
# std::shared_ptr<RigidTerrain> terrain;
# std::shared_ptr<ChSensorManager> manager;
# std::shared_ptr<ChBody> ground_body;

# std::shared_ptr<ChCameraSensor> camera;
# std::shared_ptr<ChSegmentationCamera> semantic;
# std::shared_ptr<ChGPSSensor> gps;
# std::shared_ptr<ChAccelerometerSensor> acc;
# std::shared_ptr<ChMagnetometerSensor> mag;
# std::shared_ptr<ChGyroscopeSensor> gyro;

# std::vector<std::shared_ptr<ChTriangleMeshShape>> red_cone_assets;
# std::vector<std::shared_ptr<ChTriangleMeshShape>> green_cone_assets;
# std::vector<std::shared_ptr<ChBody>> red_cones;
# std::vector<std::shared_ptr<ChBody>> green_cones;

# rclcpp::TimerBase::SharedPtr timer;

# rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image;
# rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_gps;
# rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
# rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag;
# rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pub_clock;
# rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_tracked_pose;
# rclcpp::Subscription<custom_msgs::msg::VehicleInput>::SharedPtr sub_vehicle_cmd;




#initialize simulation
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/') #from 285

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
terrainHeight = -.2339      # terrain height (FLAT terrain only)
terrainLength = 7.0  # size in X direction
terrainWidth = 7.0   # size in Y direction

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Simulation end time
t_end = 1000

# Time interval between two render frames
render_step_size = 1.0 / 30  # FPS = 30


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
patch_mat.SetRestitution(0.01) #-----check if mu, cr, and y is set similar to minfo

# not right need to introduce point cloud -- not sure how to access this using the package_share_directory
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, "../data/me3038/rm3038_pt_cloud.obj")  #chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0))
# patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
# patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
# app = veh.ChWheeledVehicleIrrApp(vehicle.GetVehicle(), 'RC Car')
# app.AddTypicalLights()
# app.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
# app.SetChaseCamera(trackPoint, 15.0, 0.5)
# app.SetTimestep(step_size)
# app.AssetBindAll()
# app.AssetUpdateAll()

driver = veh.ChDriver(app)   #ask asher about this

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.5   # time to go from 0 to +1
driver.SetSteeringDelta(2.0 * 1e-3 / steering_time)
driver.SetThrottleDelta(1.0 * 1e-3 / throttle_time)
driver.SetBrakingDelta(1.0 * 1e-3 / braking_time)

driver.Initialize()


# add in ME3038 room mesh
room_mmesh = geo.ChTriangleMeshConnected()
room_mmesh.SetLoadWavefrontMesh("../data/me3038/rm3038_pt_cloud.obj", False, True)
room_mmesh.SetTransform(chrono.ChVector(0, 0, 0), chrono.ChMatrix33(1))

room_trimesh_shape = chrono.ChTriangleMeshShape()
room_trimesh_shape.SetMesh(room_mmesh)
room_trimesh_shape.SetName("ME3038")
room_trimesh_shape.SetStatic(True)
room_mesh_body = chrono.ChBody()
room_mesh_body.SetPos((0, 0, 0))
room_mesh_body.AddAsset(room_trimesh_shape)
room_mesh_body.SetBodyFixed(True)

vehicle.GetSystem().Add(room_mesh_body)


if cones_from_file:
    AddConesFromFile()
else:
    AddRandomCones(num_cones / 2, GetChronoDataFile("sensor/cones/green_cone.obj"), 2)
    AddRandomCones(num_cones / 2, GetChronoDataFile("sensor/cones/red_cone.obj"), 1)

# === create sensors ===
manager = sensor.ChSensorManager(vehicle.GetSystem())
manager.scene.AddPointLight((100, 100, 100), (1, 1, 1), 5000)
b = sensor.Background()
b.setBackgroundMode(GRADIENT) #not sure about this
b.color_horizon = (.6, .7, .8)
b.color_zenith = (.4, .5, .6)
manager.scene.SetBackground(b)

camera_pose = chrono.ChFrame((0, 0, .2), chrono.Q_from_AngAxis(.2, (0, 1, 0)))

width = 1280
height = 720
frame_rate = 30.0
if create_semantic_maps:
    frame_rate = float(5)    # not sure about this
fov = 1.396;  # 80 degree FOV camera)

camera = chrono.ChCameraSensor(
            vehicle.GetChassisBody(), # body camera is attached to
            frame_rate,                 # update rate in Hz
            camera_pose,                # offset pose
            width,                      # image width
            height,                     # image height
            fov,                        # FOV
            2,
            sensor.CameraLensModelType(FOV_LENS))  # super sample diameter
camera.SetName("Camera Sensor")
c_window = float(0)
camera.SetCollectionWindow(c_window)

camera.PushFilter(sensor.ChFilterVisualize(640, 360))

if save_sensor_data:
    camera.PushFilter(sensor.ChFilterSave(sensor_data_dir + "cam1/"))

camera.PushFilter(sensor.ChFilterRGBA8Access())
manager.AddSensor(camera)


camera2 = chrono.ChCameraSensor(
            patch.GetGroundBody(),  # body camera is attached to
            30,                      # update rate in Hz
            chrono.ChFrame((init_loc_x + 2, init_loc_y, 10), 
            chrono.Q_from_AngAxis(CH_C_PI_2, (0, 1, 0))),  # offset pose   ------ not sure what CH_C_PI_2 is
            1280,                                                           # image width
            720,                                                            # image height
            3.14 / 4,                                                       # FOV
            2)                                                              # super sample diameter
camera2.SetName("Camera Sensor 2")
camera2.PushFilter(sensor.ChFilterVisualize(1280, 720))
if save_sensor_data:
    camera2.PushFilter(sensor.ChFilterSave(sensor_data_dir + "cam2/"))

manager.AddSensor(camera2)

noise_model = sensor.ChNoiseNone()
imu_offset_pose = chrono.ChFrame((0, 0, 0), chrono.Q_from_AngAxis(0, (1, 0, 0)))
gps_offset_pose = chrono.ChFrame((0, 0, 0), chrono.Q_from_AngAxis(0, (1, 0, 0)))
gps_reference = chrono.ChVector(-89.400, 43.070, 260.0)

# accelerometer
acc = sensor.ChAccelerometerSensor(vehicle.GetChassisBody(), 100, imu_offset_pose, noise_model)
acc.SetName("IMU - Accelerometer")
acc.PushFilter(sensor.ChFilterAccelAccess())
manager.AddSensor(acc)

# gyroscope
gyro = sensor.ChGyroscopeSensor(vehicle.GetChassisBody(), 100, imu_offset_pose, noise_model)
gyro.SetName("IMU - Gyroscope")
gyro.PushFilter(sensor.ChFilterGyroAccess())
manager.AddSensor(gyro)

# magnetometer
mag = sensor.ChMagnetometerSensor(vehicle.GetChassisBody(), 100, imu_offset_pose, noise_model, gps_reference)
mag.SetName("IMU - Magnetometer")
mag.PushFilter(sensor.ChFilterMagnetAccess())
manager.AddSensor(mag)

# gps
gps = sensor.ChGPSSensor(vehicle.GetChassisBody(), 10, gps_offset_pose, gps_reference, noise_model)
gps.SetName("GPS")
gps.PushFilter(sensor.ChFilterGPSAccess())
manager.AddSensor(gps)

# label the cones that were added to the scene with semantic information
manager.ReconstructScenes()
LabelConeAssets()
manager.ReconstructScenes()



# ---------------
# Simulation loop
# ---------------



# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter s
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while (app.GetDevice().run()) :
    time = vehicle.GetSystem().GetChTime()

    # End simulation
    if (time >= t_end):
        break

    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0) :
        app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
        app.DrawAll()
        app.EndScene()
        render_frame += 1
    
    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    app.Synchronize(driver.GetInputModeAsString(), driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    app.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

del app



def AddConesFromFile():
    file_name = "../" + cone_path_file
    green_cone_mesh = geo.ChTriangleMeshConnected()
    green_cone_mesh.SetLoadWavefrontMesh(chrono.GetChronoDataFile("sensor/cones/green_cone.obj"), False, True)  #not sure what get chronodatafile is
    green_cone_mesh.SetTransform(chrono.ChVector(0, 0, 0), chrono.ChMatrix33(1))

    red_cone_mesh = geo.ChTriangleMeshConnected()
    red_cone_mesh.SetLoadWavefrontMesh(chrono.GetChronoDataFile("sensor/cones/red_cone.obj"), False, True)  #not sure what get chronodatafile is
    red_cone_mesh.SetTransform(chrono.ChVector(0, 0, 0), chrono.ChMatrix33(1))

    cone_file = open(file_name, 'r')
    while True:
        line = cone_file.readline()
        if not line:
            break
        x_inner, y_inner, x_outer, y_outer = line.split(' ')

        pos_green_x = x_inner + cone_offset_x
        pos_green_y = y_inner + cone_offset_y
        pos_green_z = terrain.GetHeight((pos_green_x, pos_green_y, 1000))

        pos_red_x = x_outer + cone_offset_x
        pos_red_y = y_outer + cone_offset_y
        pos_red_z = terrain.GetHeight((pos_red_x, pos_red_y, 1000))

        pos_green = chrono.ChVector(pos_green_x, pos_green_y, pos_green_z)
        pos_red = chrono.ChVector(pos_red_x, pos_red_y, pos_red_z)
        rot = chrono.ChQuaternion(1, 0, 0, 0)

        green_cone_shape = chrono.ChTriangleMeshShape()
        green_cone_shape.SetMesh(green_cone_mesh)
        green_cone_shape.SetName("green_cone_shape")
        green_cone_shape.SetStatic(True)
        green_cone_assets.push_back(green_cone_shape)
        green_body = chrono.ChBody()
        green_body.SetPos(pos_green)
        green_body.SetRot(rot)
        green_body.AddAsset(green_cone_shape)
        green_body.SetBodyFixed(True)
        green_cones.push_back(green_body)
        vehicle.GetSystem().Add(green_body)

        red_cone_shape = chrono.ChTriangleMeshShape()
        red_cone_shape.SetMesh(red_cone_mesh)
        red_cone_shape.SetName("red_cone_shape")
        red_cone_shape.SetStatic(True)
        red_cone_assets.push_back(red_cone_shape)
        red_body = chrono.ChBody()
        red_body.SetPos(pos_red)
        red_body.SetRot(rot)
        red_body.AddAsset(red_cone_shape)
        red_body.SetBodyFixed(True)
        red_cones.push_back(red_body)
        vehicle.GetSystem().Add(red_body)
        
    cone_file.close()

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
            
        
def RedistributeCones():
    for cone in green_cones:
        x = (chrono.ChRandom() - .5) * cone_spread_x + cone_offset_x
        y = (chrono.ChRandom() - .5) * cone_spread_y + cone_offset_y
        z = terrain.GetHeight((x, y, 1000))
        cone.SetPos((x, y, z))

    for cone in red_cones:
        x = (chrono.ChRandom() - .5) * cone_spread_x + cone_offset_x
        y = (chrono.ChRandom() - .5) * cone_spread_y + cone_offset_y
        z = terrain.GetHeight((x, y, 1000))
        cone.SetPos((x, y, z))

def input_callback(msg): 
        this->throttle = throttle_scaling * msg.throttle  #not sure how to do this
        this->steering = steering_scaling * msg.steering
        this->braking = braking_scaling * msg.braking


