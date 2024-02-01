
#include "chrono/core/ChStream.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/artcar/ARTcar.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChIMUSensor.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChIMUSensor.h"
#include "chrono_sensor/sensors/ChGPSSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"


#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/vehicle/ChROSDriverInputsHandler.h"
#include "chrono_ros/handlers/sensor/ChROSCameraHandler.h"
#include "chrono_ros/handlers/sensor/ChROSAccelerometerHandler.h"
#include "chrono_ros/handlers/sensor/ChROSGyroscopeHandler.h"
#include "chrono_ros/handlers/sensor/ChROSMagnetometerHandler.h"
#include "chrono_ros/handlers/sensor/ChROSLidarHandler.h"
#include "chrono_ros/handlers/sensor/ChROSGPSHandler.h"

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/ChROSBodyHandler.h"


using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::sensor;
using namespace chrono::ros;
// using namespace chrono;

// using namespace chrono::vehicle;
// using namespace chrono::sensor;
// using namespace chrono::vehicle::artcar;

// =============================================================================

// Initial vehicle location and orientation
ChVector<> initLoc(0, 0, 0.12);
ChQuaternion<> initRot(1, 0, 0, 0);

enum DriverMode { DEFAULT, RECORD, PLAYBACK };
DriverMode driver_mode = DEFAULT;

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::PRIMITIVES;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::NONE;


// Collision type for chassis (PRIMITIVES, MESH, or NONE)
CollisionType chassis_collision_type = CollisionType::NONE;

// Type of tire model (RIGID, TMEASY)
TireModelType tire_model = TireModelType::TMEASY;

// Rigid terrain
RigidTerrain::PatchType terrain_model = RigidTerrain::PatchType::BOX;
double terrainHeight = 0;      // terrain height (FLAT terrain only)
double terrainLength = 100.0;  // size in X direction
double terrainWidth = 100.0;   // size in Y direction

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 0.2);

// Contact method
ChContactMethod contact_method = ChContactMethod::SMC;
bool contact_vis = false;

// Simulation step sizes
double step_size = 1e-3;
double tire_step_size = step_size;

// Simulation end time
double t_end = 1000;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Output directories
const std::string out_dir = GetChronoOutputPath() + "ARTcar";
const std::string pov_dir = out_dir + "/POVRAY";

// Debug logging
bool debug_output = false;
double debug_step_size = 1.0 / 1;  // FPS = 1

// POV-Ray output
bool povray_output = false;


int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    //std::this_thread::sleep_for(std::chrono::seconds(5));
    SetChronoDataPath(CHRONO_DATA_DIR);
    // Create the Sedan vehicle, set parameters, and initialize
    artcar::ARTcar car;
    car.SetContactMethod(contact_method);
    car.SetChassisCollisionType(chassis_collision_type);
    car.SetChassisFixed(false);
    car.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    car.SetTireType(tire_model);
    car.SetTireStepSize(tire_step_size);
    // car.SetMaxMotorVoltageRatio(0.16f);                                                                  
    // car.SetStallTorque(2.4f);
    // car.SetTireRollingResistance(0.06f); 
    car.SetMaxMotorVoltageRatio(0.09f);
    car.SetStallTorque(0.3f);
    car.SetTireRollingResistance(0.05f);
    car.Initialize();


    VisualizationType tire_vis_type = VisualizationType::PRIMITIVES;

    car.SetChassisVisualizationType(chassis_vis_type);
    car.SetSuspensionVisualizationType(suspension_vis_type);
    car.SetSteeringVisualizationType(steering_vis_type);
    car.SetWheelVisualizationType(wheel_vis_type);
    car.SetTireVisualizationType(tire_vis_type);

    // Create the terrain
    RigidTerrain terrain(car.GetSystem());

    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);

    std::shared_ptr<RigidTerrain::Patch> patch;
    switch (terrain_model) {
        case RigidTerrain::PatchType::BOX:
            patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, terrainWidth);
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
            break;
        case RigidTerrain::PatchType::HEIGHT_MAP:
            patch = terrain.AddPatch(patch_mat, CSYSNORM, vehicle::GetDataFile("terrain/height_maps/test64.bmp"),
                                     128, 128, 0, 4);
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 16, 16);
            break;
        case RigidTerrain::PatchType::MESH:
            patch = terrain.AddPatch(patch_mat, CSYSNORM, vehicle::GetDataFile("terrain/meshes/test.obj"));
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 100, 100);
            break;
    }
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));

    terrain.Initialize();

    // RigidTerrain terrain(system, vehicle::GetDataFile(rigidterrain_file));
    // terrain.Initialize();
    // Create the basic driver
    //auto driver = std::make_shared<ChDriver>(vehicle.GetVehicle());

    // Create a sensor manager
    // auto manager = chrono_types::make_shared<ChSensorManager>(system);
    // manager->scene->AddPointLight({100, 100, 100}, {0.4f, 0.4f, 0.4f}, 500);
    // Set the background to an environment map



    auto manager = chrono_types::make_shared<ChSensorManager>(car.GetSystem());
    manager->scene->AddPointLight({100, 100, 100}, {0.4f, 0.4f, 0.4f}, 500);

    


    Background b;
    b.mode = BackgroundMode::ENVIRONMENT_MAP;
    b.env_tex = GetChronoDataFile("sensor/textures/quarry_01_4k.hdr");
    manager->scene->SetBackground(b);
    manager->SetVerbose(false);

    // Add an IMU sensor (Accelerometer + Gyroscope + Magnetometer)
   // ChFrame<double> sensor_offset_pose(ChVector<>(0, 0, 0), Q_from_AngAxis(0, {1, 0, 0}));
    auto sensor_offset_pose = chrono::ChFrame<double>({0, 0, 0}, Q_from_AngAxis(0, {0, 0, 0}));
    auto noise_none = chrono_types::make_shared<ChNoiseNone>();
    ChVector<> gps_reference(-89.400, 43.070, 260.0);

   
   

    ChVector<double> gyro_noise_mean(0, 0, 0);
    ChVector<double> gyro_noise_stdev(0.05, 0.05, 0.05); // degrees per second
    ChVector<double> accel_noise_mean(0.0, 0.0, 0.0);
    ChVector<double> accel_noise_stdev(0.01, 0.01, 0.01); // m/s²
    double gyro_drift_bias = 0.02; // degrees per second
    double gyro_tau_drift = 75; // seconds
    double update_rate = 100; // Sensor update rate in Hz

 
    // auto gyro_noise_model = chrono_types::make_shared<ChNoiseNormalDrift>(
    //     update_rate, gyro_noise_mean, gyro_noise_stdev, gyro_drift_bias, gyro_tau_drift);

    auto gyro_noise_model = chrono_types::make_shared<ChNoiseNormal>(
        gyro_noise_mean, gyro_noise_stdev);
    auto accel_noise_model = chrono_types::make_shared<ChNoiseNormal>(
        accel_noise_mean, accel_noise_stdev);

    //auto accel_noise_model = chrono_types::make_shared<ChNoiseNone>();
    //auto gyro_noise_model = chrono_types::make_shared<ChNoiseNone>();
     
    // auto accel_noise_model = chrono_types::make_shared<ChNoiseNormalDrift>(
    //     update_rate, gyro_noise_mean, accel_noise_stdev, gyro_drift_bias, gyro_tau_drift);

    ChFrame<double> sensor_frame(ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0));


    // Accelerometer
    auto accelerometer = chrono_types::make_shared<ChAccelerometerSensor>(car.GetChassisBody(), 100, sensor_offset_pose, accel_noise_model);
    accelerometer->PushFilter(chrono_types::make_shared<ChFilterAccelAccess>());
    manager->AddSensor(accelerometer);

    // Gyroscope
    auto gyroscope = chrono_types::make_shared<ChGyroscopeSensor>(car.GetChassisBody(), 100, sensor_offset_pose,gyro_noise_model);
    gyroscope->PushFilter(chrono_types::make_shared<ChFilterGyroAccess>());
    manager->AddSensor(gyroscope);

    // Magnetometer
    auto magnetometer = chrono_types::make_shared<ChMagnetometerSensor>(car.GetChassisBody(), 100, sensor_offset_pose, noise_none, gps_reference);
    magnetometer->PushFilter(chrono_types::make_shared<ChFilterMagnetAccess>());
    manager->AddSensor(magnetometer);

    ChVector<double> gps_noise_mean(0, 0, 0);
    ChVector<double> gps_noise_stdev(0.017, 0.017, 0.017); // meters

    auto gps_noise_model = chrono_types::make_shared<ChNoiseNormal>(
    gps_noise_mean, gps_noise_stdev);

    auto gps = chrono_types::make_shared<ChGPSSensor>(car.GetChassis()->GetBody(), 10.f, sensor_offset_pose, gps_reference, noise_none);
    gps->PushFilter(chrono_types::make_shared<ChFilterGPSAccess>());
    manager->AddSensor(gps);
    manager->Update();



    // Create ROS manager
    auto ros_manager = chrono_types::make_shared<ChROSManager>();
    auto clock_handler = chrono_types::make_shared<ChROSClockHandler>();
    ros_manager->RegisterHandler(clock_handler);

    // Setup ROS handlers for sensors
    // auto lidar_topic_name = "~/output/lidar/data/pointcloud";
    // auto lidar_handler = chrono_types::make_shared<ChROSLidarHandler>(lidar, lidar_topic_name);
    // ros_manager->RegisterHandler(lidar_handler);

    auto accelerometer_topic_name = "~/output/accelerometer/data";
    auto accelerometer_handler = chrono_types::make_shared<ChROSAccelerometerHandler>(accelerometer, accelerometer_topic_name);
    ros_manager->RegisterHandler(accelerometer_handler);

    auto gyroscope_topic_name = "~/output/gyroscope/data";
    auto gyroscope_handler = chrono_types::make_shared<ChROSGyroscopeHandler>(gyroscope, gyroscope_topic_name);
    ros_manager->RegisterHandler(gyroscope_handler);

    auto magnetometer_topic_name = "~/output/magnetometer/data";
    auto magnetometer_handler = chrono_types::make_shared<ChROSMagnetometerHandler>(magnetometer, magnetometer_topic_name);
    ros_manager->RegisterHandler(magnetometer_handler);


    auto gps_topic_name = "~/output/gps/data";
    auto gps_handler = chrono_types::make_shared<ChROSGPSHandler>(gps, gps_topic_name);
    ros_manager->RegisterHandler(gps_handler);

    auto vehicle_state_rate = 5.0;
    auto vehicle_state_topic_name = "~/output/vehicle/state";
    auto vehicle_state_handler = chrono_types::make_shared<ChROSBodyHandler>(
        vehicle_state_rate, car.GetChassisBody(), vehicle_state_topic_name);
    ros_manager->RegisterHandler(vehicle_state_handler);

    // Initialize ROS manager
    ros_manager->Initialize();

    utils::CSV_writer csv_writer;
    std::string csv_file = "accelerometer_data_art.csv";
 
    std::ofstream file_stream;
    file_stream.open(
        csv_file, std::ofstream::out | std::ofstream::trunc);  // Open in write mode, truncating the file to zero length
    if (!file_stream.is_open()) {
        std::cerr << "Failed to open " << csv_file << " for writing." << std::endl;
        return 1;
    }

    std::ofstream acc_file_stream;
    std::string acc_file_name = "acceleration_data.txt";
    acc_file_stream.open(acc_file_name, std::ofstream::out | std::ofstream::trunc);
    if (!acc_file_stream.is_open()) {
        std::cerr << "Failed to open " << acc_file_name << " for writing." << std::endl;
        return 1;
    }


 // Create the vehicle Irrlicht interface
    // auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    // vis->SetWindowTitle("ARTcar Demo");
    // vis->SetChaseCamera(trackPoint, 1.5, 0.05);
    // vis->Initialize();
    // vis->AddLightDirectional();
    // vis->AddSkyBox();
    // vis->AddLogo();
    // vis->AttachVehicle(&car.GetVehicle());

    // -----------------
    // Initialize output
    // -----------------

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
        terrain.ExportMeshPovray(out_dir);
    }

    std::string driver_file = out_dir + "/driver_inputs.txt";
    utils::CSV_writer driver_csv(" ");

    // ------------------------
    // Create the driver system
    // ------------------------

    // Create the interactive driver system
    //ChInteractiveDriverIRR driver(*vis);

    // Set the time response for steering and throttle keyboard inputs.
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetThrottleDelta(render_step_size / throttle_time);
    driver.SetBrakingDelta(render_step_size / braking_time);

    // If in playback mode, attach the data file to the driver system and
    // force it to playback the driver inputs.
    if (driver_mode == PLAYBACK) {
        driver.SetInputDataFile(driver_file);
        driver.SetInputMode(ChInteractiveDriverIRR::InputMode::DATAFILE);
    }

    

    driver.Initialize();

    // ---------------
    // Simulation loop
    // ---------------

    if (debug_output) {
        GetLog() << "\n\n============ System Configuration ============\n";
        car.LogHardpointLocations();
    }

    // output vehicle mass
    std::cout << "VEHICLE MASS: " << car.GetVehicle().GetMass() << std::endl;

    // Number of simulation steps between miscellaneous events
    int render_steps = (int)std::ceil(render_step_size / step_size);
    int debug_steps = (int)std::ceil(debug_step_size / step_size);

    // Initialize simulation frame counters
    int step_number = 0;
    int render_frame = 0;

    // if (contact_vis) {
    //     vis->SetSymbolScale(1e-4);
    //     vis->EnableContactDrawing(ContactsDrawMode::CONTACT_FORCES);
    // }

    bool acceleration_stabilized = false;

    std::cout << acceleration_stabilized << std::endl;

    car.GetVehicle().EnableRealtime(true);

    double simulation_start_time = car.GetSystem()->GetChTime();
    double throttle_start_time = 10.0; 
    while (true) {
        double time = car.GetSystem()->GetChTime();

        //car.GetChassisBody()->SetRot(ChQuaternion<>(0, 0, 0, 0));
        //std::cout << acceleration_stabilized << std::endl;

        // End simulation
        if (time >= t_end)
            break;
            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteVisualizationAssets(car.GetSystem(), filename);
            }

            render_frame++;

            
        
            // Get driver inputs
            DriverInputs driver_inputs = driver.GetInputs();
            driver_inputs.m_throttle = 0.0;

            
            ///////// For constant throttle only //////
            //////////////////////////////////////////
            if (time - simulation_start_time > throttle_start_time) {
                driver_inputs.m_throttle = 0.5;
            } else {
                driver_inputs.m_throttle = 0.0;
                }
            ////////////////////////////////////////// 
            //////////////////////////////////////////   

            
            driver_inputs.m_steering = 0.0;
            driver_inputs.m_braking = 0.0;

            // Driver output
            if (driver_mode == RECORD) {
                driver_csv << time << driver_inputs.m_steering << driver_inputs.m_throttle << driver_inputs.m_braking
                        << std::endl;
            }

            ChVector<> sensor_position(0, 0, 0);  // Sensor position on the chassis
            ChFrame<double> sensor_frame(ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0));
            ChVector<> tran_acc_no_offset = car.GetChassisBody()->PointAccelerationLocalToParent(sensor_frame.GetPos());
            ChVector<> gravity = -car.GetChassisBody()->GetSystem()->Get_G_acc();
            ChVector<> tran_acc_offset = car.GetChassis()->GetRot().Rotate(gravity);
            ChVector<> total_acceleration = tran_acc_no_offset + tran_acc_offset;

            ChVector<> ground_truth_acc = car.GetChassisBody()->GetPos_dtdt();
            double current_x_acc = ground_truth_acc.x();

            ChVector<> ground_truth_vel = car.GetChassisBody()->GetPos_dt();
            double current_x_vel = ground_truth_vel.x();

            // Calculate difference in x-component
            double x_diff = ground_truth_acc.x() - tran_acc_no_offset.x() ;

            //std::cout << current_x_acc << std::endl;


            //Check if the acceleration has stabilized
            // if (std::abs(current_x_acc) < 1e-7 && std::abs(current_x_acc) > 0 && !acceleration_stabilized) {
            //     acceleration_stabilized = true;
                
                
            // }

            // if (acceleration_stabilized == true) {
            //     driver_inputs.m_throttle = 0.5;
                
                
            // } else {
            //     driver_inputs.m_throttle = 0.0;
            // }


    //         ChVector<double> tran_acc_no_offset = m_parent->PointAccelerationLocalToParent(m_offsetPose.GetPos());
    // ChVector<double> tran_acc_offset = -m_parent->GetSystem()->Get_G_acc();
    // tran_acc_offset = m_parent->GetRot().Rotate(tran_acc_offset);
    // m_keyframes.push_back(tran_acc_no_offset + tran_acc_offset);

            // Write acceleration data to CSV
            csv_writer << tran_acc_no_offset  << gravity << tran_acc_offset  <<  total_acceleration << x_diff << std::endl;

            
            acc_file_stream << time << "\t" << current_x_acc << "\t" << acceleration_stabilized<<  "\t" << current_x_vel << std::endl;

            std::cout << "Ground truth acceleration: " << ground_truth_acc << std::endl;
            std::cout << current_x_acc << std::endl;
            //std::cout << "Ground truth vel: " << current_x_vel << std::endl;

            // Update modules (process inputs from other modules)
            driver.Synchronize(time);
            terrain.Synchronize(time);
            car.Synchronize(time, driver_inputs, terrain);
            vis->Synchronize(time, driver_inputs);

            // Advance simulation for one timestep for all modules
            driver.Advance(step_size);
            terrain.Advance(step_size);
            car.Advance(step_size);
            //vis->Advance(step_size);

            manager->Update();

                    // Update ROS manager
            if (!ros_manager->Update(time, step_size))
                break;

            //std::cout << "Speed " << car.GetChassisBody()->GetPos_dt()[0] << std::endl;

            // Increment frame number
            step_number++;
        }

    csv_writer.write_to_file(csv_file);
    file_stream.close();
    return 0;
}

// //=============================================================================
// #include "chrono/core/ChTypes.h"
// #include "chrono/physics/ChSystemNSC.h"
// #include "chrono/physics/ChBodyEasy.h"
// #include "chrono/geometry/ChTriangleMeshConnected.h"

// #include "chrono_ros/ChROSManager.h"
// #include "chrono_ros/handlers/ChROSClockHandler.h"
// #include "chrono_ros/handlers/vehicle/ChROSDriverInputsHandler.h"
// #include "chrono_ros/handlers/sensor/ChROSLidarHandler.h"
// #include "chrono_ros/handlers/sensor/ChROSAccelerometerHandler.h"
// #include "chrono_ros/handlers/sensor/ChROSGyroscopeHandler.h"
// #include "chrono_ros/handlers/sensor/ChROSMagnetometerHandler.h"

// #include "chrono_vehicle/ChVehicleModelData.h"
// #include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
// #include "chrono_vehicle/terrain/RigidTerrain.h"
// #include "chrono_vehicle/ChDriver.h"
// #include "chrono_models/vehicle/artcar/ARTcar.h"

// #include "chrono_sensor/ChSensorManager.h"
// #include "chrono_sensor/sensors/ChLidarSensor.h"
// #include "chrono_sensor/sensors/ChIMUSensor.h"
// #include "chrono_sensor/filters/ChFilterVisualize.h"

// #include "chrono_sensor/ChSensorManager.h"
// #include "chrono_sensor/sensors/ChCameraSensor.h"
// #include "chrono_sensor/sensors/ChLidarSensor.h"
// #include "chrono_sensor/sensors/ChIMUSensor.h"
// #include "chrono_sensor/sensors/ChGPSSensor.h"
// #include "chrono_sensor/filters/ChFilterAccess.h"
// #include "chrono_sensor/filters/ChFilterPCfromDepth.h"
// #include "chrono_sensor/filters/ChFilterVisualize.h"
// #include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"

// #include "chrono/core/ChTypes.h"

// #include "chrono/core/ChRealtimeStep.h"
// #include "chrono/physics/ChSystemNSC.h"
// #include "chrono/physics/ChBody.h"
// #include "chrono/physics/ChBodyEasy.h"
// #include "chrono/geometry/ChTriangleMeshConnected.h"

// #include "chrono_ros/ChROSManager.h"
// #include "chrono_ros/handlers/ChROSClockHandler.h"
// #include "chrono_ros/handlers/vehicle/ChROSDriverInputsHandler.h"
// #include "chrono_ros/handlers/sensor/ChROSCameraHandler.h"
// #include "chrono_ros/handlers/sensor/ChROSAccelerometerHandler.h"
// #include "chrono_ros/handlers/sensor/ChROSGyroscopeHandler.h"
// #include "chrono_ros/handlers/sensor/ChROSMagnetometerHandler.h"
// #include "chrono_ros/handlers/sensor/ChROSLidarHandler.h"
// #include "chrono_ros/handlers/sensor/ChROSGPSHandler.h"

// #include "chrono_ros/ChROSManager.h"
// #include "chrono_ros/handlers/ChROSClockHandler.h"
// #include "chrono_ros/handlers/ChROSBodyHandler.h"

// #include <iostream>
// #include <fstream>
// #include <sstream>
// #include <vector>

// #include "ChNoiseModel.h"

// using namespace chrono;
// using namespace chrono::ros;
// using namespace chrono::vehicle;
// using namespace chrono::sensor;
// using namespace chrono::vehicle::artcar;

// // Initial vehicle location and orientation
// ChVector<> initLoc(0, 0, 0.0813);
// //ChVector<> initLoc_flw(0, 0.0, 0.5);
// ChQuaternion<> initRot(1, 0, 0, 0);
// // Rigid terrain
// RigidTerrain::PatchType terrain_model = RigidTerrain::PatchType::BOX;
// double terrainHeight = 0;      // terrain height (FLAT terrain only)
// double terrainLength = 10.0;  // size in X direction
// double terrainWidth = 10.0;   // size in Y direction

// // Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
// VisualizationType chassis_vis_type = VisualizationType::PRIMITIVES;
// VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
// VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
// VisualizationType wheel_vis_type = VisualizationType::NONE;

// // Contact method
// ChContactMethod contact_method = ChContactMethod::NSC;
// // Collision type for chassis (PRIMITIVES, MESH, or NONE)
// CollisionType chassis_collision_type = CollisionType::NONE;
// // Type of tire model (RIGID, TMEASY)
// TireModelType tire_model = TireModelType::TMEASY;
// // JSON files for terrain
// std::string rigidterrain_file("terrain/RigidPlane.json");
// ////std::string rigidterrain_file("terrain/RigidMesh.json");
// ////std::string rigidterrain_file("terrain/RigidHeightMap.json");
// ////std::string rigidterrain_file("terrain/RigidSlope10.json");
// ////std::string rigidterrain_file("terrain/RigidSlope20.json");


// //sensor params
// unsigned int image_width = 1080;
// unsigned int image_height = 720;
// float fov = (float)CH_C_PI / 2.;
// int alias_factor = 1;
// float lag = 0.0f;
// CameraLensModelType lens_model = CameraLensModelType::PINHOLE;
// // Simulation step size
// double step_size = 1e-3;

// int main(int argc, char* argv[]) {
//     // Set Chrono data path and vehicle parameters

//     std::this_thread::sleep_for(std::chrono::seconds(5));
//     SetChronoDataPath(CHRONO_DATA_DIR);


//     // Create the ARTcar vehicle
//     ARTcar vehicle;
//     vehicle.SetContactMethod(contact_method);
//     vehicle.SetChassisCollisionType(chassis_collision_type);
//     vehicle.SetChassisFixed(false);
//     vehicle.SetInitPosition(ChCoordsys<>(initLoc, initRot));
//     vehicle.SetTireType(tire_model);
//     vehicle.SetTireStepSize(step_size);
//     vehicle.SetMaxMotorVoltageRatio(0.16f);
//     vehicle.SetStallTorque(0.4f);
//     vehicle.SetTireRollingResistance(0.06f);
//     vehicle.Initialize();

//     // Setup terrain
//    // Create the terrain
//     RigidTerrain terrain(vehicle.GetSystem());
//     RigidTerrain::PatchType terrain_model = RigidTerrain::PatchType::BOX;
//     ChContactMaterialData minfo;
//     minfo.mu = 0.9f;
//     minfo.cr = 0.01f;
//     minfo.Y = 2e7f;
//     auto patch_mat = minfo.CreateMaterial(contact_method);
 
//     std::shared_ptr<RigidTerrain::Patch> patch;
//     switch (terrain_model) {
//         case RigidTerrain::PatchType::BOX:
//             patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, terrainWidth);
//             patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
//             break;
//         case RigidTerrain::PatchType::HEIGHT_MAP:
//             patch = terrain.AddPatch(patch_mat, CSYSNORM, vehicle::GetDataFile("terrain/height_maps/test64.bmp"),
//                                      128, 128, 0, 4);
//             patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 16, 16);
//             break;
//         case RigidTerrain::PatchType::MESH:
//             patch = terrain.AddPatch(patch_mat, CSYSNORM, vehicle::GetDataFile("terrain/meshes/test.obj"));
//             patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 100, 100);
//             break;
//     }
//     patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
 
//     terrain.Initialize();

//     auto manager = chrono_types::make_shared<ChSensorManager>(vehicle.GetSystem());
//     manager->scene->AddPointLight({100, 100, 100}, {0.4f, 0.4f, 0.4f}, 500);

    


//     Background b;
//     b.mode = BackgroundMode::ENVIRONMENT_MAP;
//     b.env_tex = GetChronoDataFile("sensor/textures/quarry_01_4k.hdr");
//     manager->scene->SetBackground(b);
//     manager->SetVerbose(false);

//     // Add an IMU sensor (Accelerometer + Gyroscope + Magnetometer)
//     ChFrame<double> sensor_offset_pose(ChVector<>(0, 0, 0), Q_from_AngAxis(0, {1, 0, 0}));
//     auto noise_none = chrono_types::make_shared<ChNoiseNone>();
//     ChVector<> gps_reference(-89.400, 43.070, 260.0);

//     ChVector<double> gyro_noise_mean(0, 0, 0);
//     ChVector<double> gyro_noise_stdev(0.00, 0.00, 0.00); // degrees per second
//     ChVector<double> accel_noise_mean(0, 0, 0);
//     ChVector<double> accel_noise_stdev(0.0, 0.0, 0.0); // m/s²
//     double gyro_drift_bias = 0.02; // degrees per second
//     double gyro_tau_drift = 75; // seconds
//     double update_rate = 100; // Sensor update rate in Hz

 
//     // auto gyro_noise_model = chrono_types::make_shared<ChNoiseNormalDrift>(
//     //     update_rate, gyro_noise_mean, gyro_noise_stdev, gyro_drift_bias, gyro_tau_drift);

//     auto gyro_noise_model = chrono_types::make_shared<ChNoiseNormal>(
//         gyro_noise_mean, gyro_noise_stdev);
//     auto accel_noise_model = chrono_types::make_shared<ChNoiseNormal>(
//         accel_noise_mean, accel_noise_stdev);
     
//     // auto accel_noise_model = chrono_types::make_shared<ChNoiseNormalDrift>(
//     //     update_rate, gyro_noise_mean, accel_noise_stdev, gyro_drift_bias, gyro_tau_drift);


//     // Accelerometer
//     auto accelerometer = chrono_types::make_shared<ChAccelerometerSensor>(vehicle.GetChassis()->GetBody(), 100, sensor_offset_pose, noise_none);
//     accelerometer->PushFilter(chrono_types::make_shared<ChFilterAccelAccess>());
//     manager->AddSensor(accelerometer);

//     // Gyroscope
//     auto gyroscope = chrono_types::make_shared<ChGyroscopeSensor>(vehicle.GetChassis()->GetBody(), 100, sensor_offset_pose, noise_none);
//     gyroscope->PushFilter(chrono_types::make_shared<ChFilterGyroAccess>());
//     manager->AddSensor(gyroscope);

//     // Magnetometer
//     auto magnetometer = chrono_types::make_shared<ChMagnetometerSensor>(vehicle.GetChassis()->GetBody(), 100, sensor_offset_pose, noise_none, gps_reference);
//     magnetometer->PushFilter(chrono_types::make_shared<ChFilterMagnetAccess>());
//     manager->AddSensor(magnetometer);

//     ChVector<double> gps_noise_mean(0, 0, 0);
//     ChVector<double> gps_noise_stdev(0.00, 0.00, 0.00); // meters

//     auto gps_noise_model = chrono_types::make_shared<ChNoiseNormal>(
//     gps_noise_mean, gps_noise_stdev);

//     auto gps = chrono_types::make_shared<ChGPSSensor>(vehicle.GetChassis()->GetBody(), 10.f, sensor_offset_pose, gps_reference, noise_none);
//     gps->PushFilter(chrono_types::make_shared<ChFilterGPSAccess>());
//     manager->AddSensor(gps);
//     manager->Update();


//     // Create ROS manager
//     auto ros_manager = chrono_types::make_shared<ChROSManager>();
//     auto clock_handler = chrono_types::make_shared<ChROSClockHandler>();
//     ros_manager->RegisterHandler(clock_handler);

//     // Setup ROS handlers for sensors
//     // auto lidar_topic_name = "~/output/lidar/data/pointcloud";
//     // auto lidar_handler = chrono_types::make_shared<ChROSLidarHandler>(lidar, lidar_topic_name);
//     // ros_manager->RegisterHandler(lidar_handler);

//     auto accelerometer_topic_name = "~/output/accelerometer/data";
//     auto accelerometer_handler = chrono_types::make_shared<ChROSAccelerometerHandler>(accelerometer, accelerometer_topic_name);
//     ros_manager->RegisterHandler(accelerometer_handler);

//     auto gyroscope_topic_name = "~/output/gyroscope/data";
//     auto gyroscope_handler = chrono_types::make_shared<ChROSGyroscopeHandler>(gyroscope, gyroscope_topic_name);
//     ros_manager->RegisterHandler(gyroscope_handler);

//     auto magnetometer_topic_name = "~/output/magnetometer/data";
//     auto magnetometer_handler = chrono_types::make_shared<ChROSMagnetometerHandler>(magnetometer, magnetometer_topic_name);
//     ros_manager->RegisterHandler(magnetometer_handler);



//     auto gps_topic_name = "~/output/gps/data";
//     auto gps_handler = chrono_types::make_shared<ChROSGPSHandler>(gps, gps_topic_name);
//     ros_manager->RegisterHandler(gps_handler);

//     auto vehicle_state_rate = 25;
//     auto vehicle_state_topic_name = "~/output/vehicle/state";
//     auto vehicle_state_handler = chrono_types::make_shared<ChROSBodyHandler>(
//         vehicle_state_rate, vehicle.GetChassisBody(), vehicle_state_topic_name);
//     ros_manager->RegisterHandler(vehicle_state_handler);

//     // Initialize ROS manager
//     ros_manager->Initialize();

//     // Simulation loop
//     double step_size = 1e-3;
//     double time_end = 300; // 30 seconds simulation
//     double throttle = 0.0f; // Constant throttle value

//     for (double time = 0; time < time_end; time += step_size) {
//         time = vehicle.GetSystem()->GetChTime();
//         // Set constant throttle
//         DriverInputs driver_inputs;
//         driver_inputs.m_throttle = throttle;
//         driver_inputs.m_steering = 0.0f;
//         driver_inputs.m_braking = 0.0f;

//         // Update vehicle
//         terrain.Synchronize(time);
//         vehicle.Synchronize(time, driver_inputs, terrain);
        
//         terrain.Advance(step_size);
//         vehicle.Advance(step_size);
//         std::cout << "Speed " << vehicle.GetChassisBody()->GetPos_dt()[0] << std::endl;
//         // Update sensors
//         manager->Update();

//         // Update ROS manager
//         if (!ros_manager->Update(time, step_size))
//             break;

//         // Step the system
//         // vehicle.GetSystem()->DoStepDynamics(step_size);
//     }

//     return 0;
// }


