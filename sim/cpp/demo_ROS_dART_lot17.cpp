// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Harry Zhang   Aaron Young
// =============================================================================
//
// Demo to show the use of Chrono::Vehicle with ROS
//
// =============================================================================

#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>

#include "chrono/core/ChTypes.h"

#include "chrono/physics/ChSystemNSC.h"
//#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChBody.h"

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/ChROSBodyHandler.h"
#include "chrono_ros/handlers/vehicle/ChROSDriverInputsHandler.h"
#include "chrono_ros/handlers/sensor/ChROSLidarHandler.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/ChDriver.h"
//#include "chrono_models/vehicle/artvehicle/ARTvehicle.h"
#include "chrono_models/vehicle/artcar/ARTcar.h"


#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/sensors/ChIMUSensor.h"
#include "chrono_sensor/sensors/ChGPSSensor.h"

#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"
#include "chrono_sensor/filters/ChFilterLidarReduce.h"
#include "chrono_sensor/filters/ChFilterLidarNoise.h"
#include "chrono_sensor/filters/ChFilterSavePtCloud.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"
#include "chrono_sensor/sensors/Sensor.h"
#include "chrono_sensor/filters/ChFilterRadarVisualizeCluster.h"
#include "chrono_sensor/filters/ChFilterRadarXYZReturn.h"
#include "chrono_sensor/filters/ChFilterRadarXYZVisualize.h"
#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/vehicle/ChROSDriverInputsHandler.h"
#include "chrono_ros/handlers/sensor/ChROSCameraHandler.h"
#include "chrono_ros/handlers/sensor/ChROSAccelerometerHandler.h"
#include "chrono_ros/handlers/sensor/ChROSGyroscopeHandler.h"
#include "chrono_ros/handlers/sensor/ChROSMagnetometerHandler.h"
#include "chrono_ros/handlers/sensor/ChROSLidarHandler.h"
#include "chrono_ros/handlers/sensor/ChROSGPSHandler.h"
#include "chrono_ros/handlers/sensor/ChROSAirSimGPSHandler.h"

#include <chrono>
#include <random>
using namespace chrono;
using namespace chrono::ros;
using namespace chrono::vehicle;
using namespace chrono::sensor;
//using namespace chrono::vehicle::artvehicle;
// =============================================================================
// Initial vehicle location and orientation
ChVector<> initLoc(0.0, 0.0, 0.5);
ChVector<> initLoc_flw(0, 0.0, 0.5);
ChQuaternion<> initRot = Q_from_AngZ(0.0f);
// Rigid terrain
RigidTerrain::PatchType terrain_model = RigidTerrain::PatchType::BOX;
double terrainHeight = 0;      // terrain height (FLAT terrain only)
double terrainLength = 10.0;  // size in X direction
double terrainWidth = 10.0;   // size in Y direction

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::PRIMITIVES;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::NONE;

// Contact method
ChContactMethod contact_method = ChContactMethod::NSC;
// Collision type for chassis (PRIMITIVES, MESH, or NONE)
CollisionType chassis_collision_type = CollisionType::NONE;
// Type of tire model (RIGID, TMEASY)
TireModelType tire_model = TireModelType::TMEASY;
// JSON files for terrain
std::string rigidterrain_file("terrain/RigidPlane.json");
////std::string rigidterrain_file("terrain/RigidMesh.json");
////std::string rigidterrain_file("terrain/RigidHeightMap.json");
////std::string rigidterrain_file("terrain/RigidSlope10.json");
////std::string rigidterrain_file("terrain/RigidSlope20.json");

//sensor params
unsigned int image_width = 1080;
unsigned int image_height = 720;
float fov = (float)CH_C_PI / 2.;
int alias_factor = 1;
float lag = 0.0f;
CameraLensModelType lens_model = CameraLensModelType::PINHOLE;
// Simulation step size
double step_size = 1e-3;

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2023 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    //SetChronoDataPath(CHRONO_DATA_DIR);
    artcar::ARTcar vehicle;
    //vehicle::SetDataPath(std::string(CHRONO_DATA_DIR) + "/vehicle/");
    vehicle.SetContactMethod(contact_method);
    vehicle.SetChassisCollisionType(chassis_collision_type);
    vehicle.SetChassisFixed(false);
    vehicle.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    vehicle.SetTireType(tire_model);
    vehicle.SetTireStepSize(step_size);
    vehicle.SetMaxMotorVoltageRatio(0.18f);
    vehicle.SetStallTorque(0.5f);
    vehicle.SetTireRollingResistance(0.05f);
    vehicle.Initialize();

    VisualizationType tire_vis_type = VisualizationType::MESH;
    std::cout << "Initialization completed 1." << std::endl;

    vehicle.SetChassisVisualizationType(chassis_vis_type);
    vehicle.SetSuspensionVisualizationType(suspension_vis_type);
    vehicle.SetSteeringVisualizationType(steering_vis_type);
    vehicle.SetWheelVisualizationType(wheel_vis_type);
    vehicle.SetTireVisualizationType(tire_vis_type);


    // Containing system
    auto system = vehicle.GetSystem();

    system->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
    // system->SetTimestepperType(ChTimestepper::Type::HHT);
    // auto integrator = std::static_pointer_cast<ChTimestepperHHT>(system->GetTimestepper());
    // integrator->SetAlpha(-0.2);
    // integrator->SetMaxiters(8);
    // integrator->SetAbsTolerances(1e-1, 10);
    // integrator->SetModifiedNewton(true);
    // integrator->SetVerbose(true);

    // Add box in front of the vehicle
    auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat->SetAmbientColor({0.f, 0.f, 0.f});
    vis_mat->SetDiffuseColor({1.0, 0.0, 0.0});
    vis_mat->SetSpecularColor({1.f, 1.f, 1.f});
    vis_mat->SetUseSpecularWorkflow(true);
    vis_mat->SetRoughness(.5f);
    vis_mat->SetClassID(30000);
    vis_mat->SetInstanceID(50000);

    // Add path to follow:
    // Function to read and parse the CSV file
    std::vector<std::tuple<double, double, double, double>> positions;
    // std::string directoryPath = "/home/art/art/sim/data/autonomy-toolkit/paths/lot17_sinsquare.csv";
    std::string csvFile =  "/home/art/art/sim/data/autonomy-toolkit/paths/path.csv"; // Replace with CSV file path
    std::ifstream file(csvFile);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open CSV file." << std::endl;
        return 1;
    }
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        double x, y, z, w;
        char delimiter;
        if (ss >> x >> delimiter >> y >> delimiter >> z >> delimiter >> w) {
            positions.emplace_back(x, y, z, w);
        } else {
            std::cerr << "Error: Invalid CSV format in line: " << line << std::endl;
            return 1;
        }
    }
    // Add color to path
    // Define visual material (replace with your material setup)
    auto vis_mat_path = chrono_types::make_shared<chrono::ChVisualMaterial>();
    vis_mat_path->SetDiffuseColor(chrono::ChColor(0.0f, 1.0f, 0.0f));
    // Create ChBodyEasyBox objects at specified positions
    for (const auto& pos : positions) {
        auto box_body = chrono_types::make_shared<chrono::ChBodyEasyBox>(0.1, 0.1, 0.0001, 1000, true, false);
        box_body->SetPos(chrono::ChVector<>(std::get<0>(pos), std::get<1>(pos), 0));
        box_body->SetBodyFixed(true);

        // Set visual material for the box
        auto shape = box_body->GetVisualModel()->GetShapes()[0].first;
        if (shape->GetNumMaterials() == 0) {
            shape->AddMaterial(vis_mat_path);
        } else {
            shape->GetMaterials()[0] = vis_mat_path;
        }

        system->Add(box_body);
    }

    std::cout << "Initialization completed 2." << std::endl;

    // Create the terrain
    RigidTerrain terrain(system, vehicle::GetDataFile(rigidterrain_file));
    terrain.Initialize();

    // Create the basic driver
    auto driver = std::make_shared<ChDriver>(vehicle.GetVehicle());

    // Create a sensor manager
    auto manager = chrono_types::make_shared<ChSensorManager>(system);
    manager->scene->AddPointLight({100, 100, 100}, {0.4f, 0.4f, 0.4f}, 500);
    // Set the background to an environment map
    Background b;
    b.mode = BackgroundMode::ENVIRONMENT_MAP;
    b.env_tex = GetChronoDataFile("sensor/textures/quarry_01_4k.hdr");
    manager->scene->SetBackground(b);
    manager->SetVerbose(false);
    std::cout << "Initialization completed 3." << std::endl;


    //Add camera
    auto cam_pose = chrono::ChFrame<double>({-5.304, 0, 1.0}, Q_from_AngAxis(0.1, {0, 1.25, 0}));
    auto cam = chrono_types::make_shared<ChCameraSensor>(vehicle.GetChassis()->GetBody(),         // body camera is attached to
                                                            10,   // update rate in Hz
                                                            cam_pose,  // offset pose
                                                            image_width,   // image width
                                                            image_height,  // image height
                                                            fov,           // camera's horizontal field of view
                                                            alias_factor,  // supersample factor for antialiasing
                                                            lens_model,    // FOV
                                                            false);        // use global illumination or not

    
    cam->SetName("Camera");
    cam->SetLag(lag);
    cam->SetCollectionWindow(0.0f);
    cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Camera"));
    std::cout << "Initialization completed 4." << std::endl;
    //cam->PushFilter(chrono_types::make_shared<ChFilterSave>("./cam1/"));
    
    manager->AddSensor(cam);
    std::cout << "Initialization completed 5." << std::endl;
    // manager->Update();
    //------------

    // auto cam = chrono_types::make_shared<ChCameraSensor>(vehicle.GetChassis()->GetBody(), 30, chrono::ChFrame<double>({-5.304, 0, 1.0}, Q_from_AngAxis(0.1, {0, 1.25, 0})) , 1280, 720, CH_C_PI / 3.);
    // cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    // cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(1280, 720));
    // std::cout << "Initialization completed 5." << std::endl;
    // manager->AddSensor(cam);
    

    auto imu_offset_pose = chrono::ChFrame<double>({0, 0, 0}, Q_from_AngAxis(CH_C_PI, {1, 0, 0}));
    auto sensor_offset_pose = chrono::ChFrame<double>({0, 0, 0}, Q_from_AngAxis(0, {1, 0, 0}));
    //auto sensor_offset_pose = chrono::ChFrame<double>({0, 0, 0}, Q_from_AngAxis(CH_C_PI, {1, 0, 0}));
    auto noise_none = chrono_types::make_shared<ChNoiseNone>();
    ChVector<> gps_reference(-89.400, 43.070, 260.0);

    ChVector<double> gyro_noise_mean(0, 0, 0);
    ChVector<double> gyro_noise_stdev(0.05, 0.05, 0.05); // degrees per second
    ChVector<double> accel_noise_mean(0.0, 0.0, 0.0);
    ChVector<double> accel_noise_stdev(0.30, 0.30, 0.01); // m/s²
    std::cout << "Initialization completed 5.1." << std::endl;



    auto gyro_noise_model = chrono_types::make_shared<ChNoiseNormalDrift>(100,
        gyro_noise_mean, gyro_noise_stdev, 0.0001, 0.1);
    auto accel_noise_model = chrono_types::make_shared<ChNoiseNormalDrift>(100,
        accel_noise_mean, accel_noise_stdev, 0.1, 0.1);

    // auto gyro_noise_model = chrono_types::make_shared<ChNoiseNormal>(
    //     gyro_noise_mean, gyro_noise_stdev);
    // auto accel_noise_model = chrono_types::make_shared<ChNoiseNormal>(
    //     accel_noise_mean, accel_noise_stdev);

    auto mag_noise_model = chrono_types::make_shared<ChNoiseNormal>(ChVector<double>({0., 0., 0.}),            // float mean,
                                                         ChVector<double>({0.001, 0.001, 0.001}));


    ChFrame<double> sensor_frame(ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0));

   



    // Accelerometer
    auto accelerometer = chrono_types::make_shared<ChAccelerometerSensor>(vehicle.GetChassisBody(), 100, sensor_offset_pose, accel_noise_model);
    accelerometer->PushFilter(chrono_types::make_shared<ChFilterAccelAccess>());
    manager->AddSensor(accelerometer);

    // Gyroscope
    auto gyroscope = chrono_types::make_shared<ChGyroscopeSensor>(vehicle.GetChassisBody(), 100, sensor_offset_pose, noise_none);
    gyroscope->PushFilter(chrono_types::make_shared<ChFilterGyroAccess>());
    manager->AddSensor(gyroscope);

    // Magnetometer
    auto magnetometer = chrono_types::make_shared<ChMagnetometerSensor>(vehicle.GetChassisBody(), 100, sensor_offset_pose, mag_noise_model, gps_reference);
    magnetometer->PushFilter(chrono_types::make_shared<ChFilterMagnetAccess>());
    manager->AddSensor(magnetometer);

    ChVector<double> gps_noise_mean(0, 0, 0);
    ChVector<double> gps_noise_stdev(0.03, 0.03, 0.03); // meters

    auto gps_noise_model = chrono_types::make_shared<ChNoiseNormal>(
    gps_noise_mean, gps_noise_stdev);

    ChVector<> gps_reference1(-89.412240758, 43.071986683, 0);


   
    std::cout << "Initialization completed 5.2" << std::endl;

    auto random_walk_noise =
        chrono_types::make_shared<ChNoiseRandomWalks>(0,                             // mean
                                                      0.047,                         // std
                                                      100,  // Model Update rate
                                                      0.016,                         // max velocity
                                                      0.152,                         // max acc
                                                      gps_reference1);

    auto gps = chrono_types::make_shared<ChGPSSensor>(vehicle.GetChassis()->GetBody(), 10.f, sensor_offset_pose, gps_reference, noise_none);
    //auto gps = chrono_types::make_shared<ChGPSSensor>(vehicle.GetChassis()->GetBody(), 10.f, sensor_offset_pose, gps_reference, random_walk_noise);
    // auto gps1 = chrono_types::make_shared<ChGPSSensor>(vehicle.GetChassis()->GetBody(), 10.f, sensor_offset_pose, gps_reference, noise_none);
    // auto gps2 = chrono_types::make_shared<ChGPSSensor>(vehicle.GetChassis()->GetBody(), 10.f, sensor_offset_pose, gps_reference, gps_noise_model);
    // auto gps3 = chrono_types::make_shared<ChGPSSensor>(vehicle.GetChassis()->GetBody(), 10.f, sensor_offset_pose, gps_reference, random_walk_noise);
    


    
    gps->PushFilter(chrono_types::make_shared<ChFilterGPSAccess>());
    manager->AddSensor(gps);
    manager->Update();

    // Create ROS manager
    auto ros_manager = chrono_types::make_shared<ChROSManager>();

    // Create a publisher for the simulation clock
    // The clock automatically publishes on every tick and on topic /clock
    auto clock_handler = chrono_types::make_shared<ChROSClockHandler>();
    ros_manager->RegisterHandler(clock_handler);

    std::cout << "Initialization completed 5.3" << std::endl;


    // Create a subscriber to the driver inputs
    auto driver_inputs_rate = 10;
    auto driver_inputs_topic_name = "~/input/driver_inputs";
    auto driver_inputs_handler =
        chrono_types::make_shared<ChROSDriverInputsHandler>(driver_inputs_rate, driver, driver_inputs_topic_name);
    ros_manager->RegisterHandler(driver_inputs_handler);

    auto accelerometer_topic_name = "~/output/accelerometer/data";
    auto accelerometer_handler = chrono_types::make_shared<ChROSAccelerometerHandler>(accelerometer, accelerometer_topic_name);
    ros_manager->RegisterHandler(accelerometer_handler);

    auto gyroscope_topic_name = "~/output/gyroscope/data";
    auto gyroscope_handler = chrono_types::make_shared<ChROSGyroscopeHandler>(gyroscope, gyroscope_topic_name);
    ros_manager->RegisterHandler(gyroscope_handler);

    auto magnetometer_topic_name = "~/output/magnetometer/data";
    auto magnetometer_handler = chrono_types::make_shared<ChROSMagnetometerHandler>(magnetometer, magnetometer_topic_name);
    ros_manager->RegisterHandler(magnetometer_handler);

    
    // take command line input for hdop
    //float hdop = std::stof(argv[1]);

    auto gps_topic_name = "~/output/gps/data";
    
    //auto gps_handler = chrono_types::make_shared<ChROSAirSimGPSHandler>(gps, gps_topic_name, 100, 100,0.85,  0.1, 12.4);
    //auto gps_handler = chrono_types::make_shared<ChROSAirSimGPSHandler>(gps, gps_topic_name, 100, 100, 0.85,  0.1, 12);
    
    auto gps_handler = chrono_types::make_shared<ChROSGPSHandler>(gps, gps_topic_name);
    ros_manager->RegisterHandler(gps_handler);

    auto gps1 = chrono_types::make_shared<ChGPSSensor>(vehicle.GetChassis()->GetBody(), 10.f, sensor_offset_pose, gps_reference, noise_none);
    auto gps2 = chrono_types::make_shared<ChGPSSensor>(vehicle.GetChassis()->GetBody(), 10.f, sensor_offset_pose, gps_reference, gps_noise_model);
    auto gps3 = chrono_types::make_shared<ChGPSSensor>(vehicle.GetChassis()->GetBody(), 10.f, sensor_offset_pose, gps_reference, random_walk_noise);

    gps1->PushFilter(chrono_types::make_shared<ChFilterGPSAccess>());
    manager->AddSensor(gps1);
    manager->Update();

    gps2->PushFilter(chrono_types::make_shared<ChFilterGPSAccess>());
    manager->AddSensor(gps2);
    manager->Update();

    gps3->PushFilter(chrono_types::make_shared<ChFilterGPSAccess>());
    manager->AddSensor(gps3);
    manager->Update();


    // Create ROS handler for GPS 1
    auto gps1_topic_name = "~/output/gps/gps1";
    auto gps1_ros_handler = chrono_types::make_shared<ChROSGPSHandler>(gps1, gps1_topic_name);
    ros_manager->RegisterHandler(gps1_ros_handler);

    // Create AirSim handler for GPS 1
    auto gps1_airsim_topic_name = "~/output/gps/gps1_airsim";
    auto gps1_airsim_handler = chrono_types::make_shared<ChROSAirSimGPSHandler>(gps1, gps1_airsim_topic_name, 100, 100, 0.85, 0.1, 12.4);
    ros_manager->RegisterHandler(gps1_airsim_handler);

    auto gps1_airsim_default_topic_name = "~/output/gps/gps1_airsim_default";
    auto gps1_airsim_handler_default = chrono_types::make_shared<ChROSAirSimGPSHandler>(gps1, gps1_airsim_topic_name, 100, 100, 0.1, 0.1, 0.9);
    ros_manager->RegisterHandler(gps1_airsim_handler_default);

    // Create ROS handler for GPS 2
    auto gps2_topic_name = "~/output/gps/gps2";
    auto gps2_ros_handler = chrono_types::make_shared<ChROSGPSHandler>(gps2, gps2_topic_name);
    ros_manager->RegisterHandler(gps2_ros_handler);

    // Create AirSim handler for GPS 2
    auto gps2_airsim_topic_name = "~/output/gps/gps2_airsim";
    auto gps2_airsim_handler = chrono_types::make_shared<ChROSAirSimGPSHandler>(gps2, gps2_airsim_topic_name, 100, 100, 0.85, 0.1, 12.4);
    ros_manager->RegisterHandler(gps2_airsim_handler);

    auto gps2_airsim_default_topic_name = "~/output/gps/gps2_airsim_default";
    auto gps2_airsim_handler_default = chrono_types::make_shared<ChROSAirSimGPSHandler>(gps2, gps2_airsim_topic_name, 100, 100, 0.1, 0.1, 0.9);
    ros_manager->RegisterHandler(gps2_airsim_handler_default);

    // Create ROS handler for GPS 3
    auto gps3_topic_name = "~/output/gps/gps3";
    auto gps3_ros_handler = chrono_types::make_shared<ChROSGPSHandler>(gps3, gps3_topic_name);
    ros_manager->RegisterHandler(gps3_ros_handler);

    // Create AirSim handler for GPS 3
    auto gps3_airsim_topic_name = "~/output/gps/gps3_airsim";
    auto gps3_airsim_handler = chrono_types::make_shared<ChROSAirSimGPSHandler>(gps3, gps3_airsim_topic_name, 100, 100, 0.85, 0.1, 12.4);
    ros_manager->RegisterHandler(gps3_airsim_handler);

    auto gps3_airsim_default_topic_name = "~/output/gps/gps3_airsim_default";
    auto gps3_airsim_handler_default = chrono_types::make_shared<ChROSAirSimGPSHandler>(gps3, gps3_airsim_topic_name, 100, 100, 0.1, 0.1, 0.9);
    ros_manager->RegisterHandler(gps3_airsim_handler_default);


    // Create a publisher for the vehicle state
    auto vehicle_state_rate = 5;
    auto vehicle_state_topic_name = "~/output/vehicle/state";
    auto vehicle_state_handler = chrono_types::make_shared<ChROSBodyHandler>(
        vehicle_state_rate, vehicle.GetChassisBody(), vehicle_state_topic_name);
    ros_manager->RegisterHandler(vehicle_state_handler);

    

    // Finally, initialize the ros manager
    ros_manager->Initialize();

    double t_end = 20;
    double time = 0;
    double interval = 2;
    // Simulation loop
    
    std::cout << "Initialization completed 6." << std::endl;

    while (time < t_end) {

        // ros_manager->Update(time,step_size);
        // // hardcode random data for one vehicle:
        // driver_flw->SetThrottle(0.6f);
        // driver_flw->SetSteering(0.4f);

        // Get driver inputs
        DriverInputs driver_inputs = driver->GetInputs();
        // Update modules (process inputs from other modules)
        time = vehicle.GetSystem()->GetChTime();
        driver->Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, terrain);
        terrain.Synchronize(time);

        // Advance simulation for one timestep for all modules
        driver->Advance(step_size);
        vehicle.Advance(step_size);
        terrain.Advance(step_size);

        //std::cout << "Z Accel: " << vehicle.GetSystem()->Get_G_acc() << std::endl;
        std::cout << "X Acc: " << vehicle.GetChassisBody()->GetPos_dtdt().x() << std::endl;
    	//std::cout << "Initialization completed 7." << std::endl;
        // update sensor manager
        manager->Update();

   	 //std::cout << "Initialization completed 8." << std::endl;
        if (!ros_manager->Update(time, step_size))
            break;
    }

    return 0;
}

   