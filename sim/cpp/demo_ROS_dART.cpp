// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Harry Zhang, Aaron Young, Radu Serban
// =============================================================================
//
// Demonstration of simulating two vehicles simultaneously.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChDataDriver.h"

#include "chrono_models/vehicle/artcar/ARTcar.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_VSG
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/handlers/vehicle/ChROSDriverInputsHandler.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/ChROSBodyHandler.h"
#include "chrono_ros/handlers/sensor/ChROSCameraHandler.h"
#include "chrono_ros/handlers/sensor/ChROSAccelerometerHandler.h"
#include "chrono_ros/handlers/sensor/ChROSGyroscopeHandler.h"
#include "chrono_ros/handlers/sensor/ChROSMagnetometerHandler.h"
#include "chrono_ros/handlers/sensor/ChROSLidarHandler.h"
#include "chrono_ros/handlers/sensor/ChROSGPSHandler.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChIMUSensor.h"
#include "chrono_sensor/sensors/ChGPSSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::artcar;
using namespace chrono::ros;

// =============================================================================

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::IRRLICHT;

// Simulation step sizes
double step_size = 2e-3;
using namespace chrono::sensor;

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // --------------
    // Create systems
    // --------------
    // vehicle::SetDataPath("/opt/chrono/share/chrono/data/vehicle/" + "/vehicle/");
    chrono::SetChronoDataPath("/opt/chrono/share/chrono/data/");
    // Chrono system
    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    sys.GetSolver()->AsIterative()->SetMaxIterations(150);
    sys.SetMaxPenetrationRecoverySpeed(4.0);

    // Create the terrain
    RigidTerrain terrain(&sys);
    auto patch_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    patch_mat->SetFriction(0.9f);
    patch_mat->SetRestitution(0.01f);
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, 200, 100);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    // patch->SetTexture(vehicle::GetDataFile("terrain/textures/cubetexture_pinkwhite.png"), 200, 200);
    terrain.Initialize();

    // define ROS handlers' rate
    auto driver_inputs_rate = 25;
    auto vehicle_state_rate = 25;
    // define ROS topics' name for driver inputs and vehicle state
    auto driver_inputs_topic_name = "~/input/driver_inputs";
    auto vehicle_state_topic_name = "~/output/vehicle/state";

    // Create and initialize the first vehicle (ARTcar)
    ARTcar artcar_1(&sys);
    artcar_1.SetInitPosition(ChCoordsys<>(ChVector3d(0, 0, 0.2), QUNIT));
    artcar_1.Initialize();
    artcar_1.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    artcar_1.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    artcar_1.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    artcar_1.SetWheelVisualizationType(VisualizationType::NONE);
    artcar_1.SetTireVisualizationType(VisualizationType::PRIMITIVES);

    // Create the basic driver for the first vehicle
    auto driver_1 = std::make_shared<ChDriver>(artcar_1.GetVehicle());
    // Create ROS manager for vehicle one
    auto ros_manager_1 = chrono_types::make_shared<ChROSManager>("artcar_1");
    // Create a publisher for the simulation clock
    // The clock automatically publishes on every tick and on topic /clock
    auto clock_handler_1 = chrono_types::make_shared<ChROSClockHandler>();
    ros_manager_1->RegisterHandler(clock_handler_1);
    // Create a subscriber to the driver inputs
    auto driver_inputs_handler_1 =
        chrono_types::make_shared<ChROSDriverInputsHandler>(driver_inputs_rate, driver_1, driver_inputs_topic_name);
    ros_manager_1->RegisterHandler(driver_inputs_handler_1);
    // Create a publisher for the vehicle state
    auto vehicle_state_handler_1 = chrono_types::make_shared<ChROSBodyHandler>(
        vehicle_state_rate, artcar_1.GetChassisBody(), vehicle_state_topic_name);
    ros_manager_1->RegisterHandler(vehicle_state_handler_1);
    

    // ------------

    // Create the sensors
    auto noise_none = chrono_types::make_shared<ChNoiseNone>();
    chrono::ChFrame<double> offset_pose({-8, 0, 2}, QuatFromAngleAxis(.2, {0, 1, 0}));

    auto sensor_manager = chrono_types::make_shared<ChSensorManager>(&sys);
    sensor_manager->scene->AddPointLight({100, 100, 100}, {2, 2, 2}, 500);
    sensor_manager->scene->SetAmbientLight({0.1f, 0.1f, 0.1f});

    // auto cam = chrono_types::make_shared<ChCameraSensor>(artcar_1.GetChassisBody(), 30, offset_pose, 1280, 720, CH_PI / 3.);
    // cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    // cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(1280, 720));
    // sensor_manager->AddSensor(cam);

    // add an accelerometer, gyroscope, and magnetometer
    ChVector3d gps_reference(-89.400, 43.070, 260.0);
    auto acc = chrono_types::make_shared<ChAccelerometerSensor>(artcar_1.GetChassisBody(), 100.f, offset_pose, noise_none);
    acc->PushFilter(chrono_types::make_shared<ChFilterAccelAccess>());
    sensor_manager->AddSensor(acc);

    auto gyro = chrono_types::make_shared<ChGyroscopeSensor>(artcar_1.GetChassisBody(), 100.f, offset_pose, noise_none);
    gyro->PushFilter(chrono_types::make_shared<ChFilterGyroAccess>());
    sensor_manager->AddSensor(gyro);

    auto mag =
        chrono_types::make_shared<ChMagnetometerSensor>(artcar_1.GetChassisBody(), 100.f, offset_pose, noise_none, gps_reference);
    mag->PushFilter(chrono_types::make_shared<ChFilterMagnetAccess>());
    sensor_manager->AddSensor(mag);

    // add a GPS sensor
    auto gps = chrono_types::make_shared<ChGPSSensor>(artcar_1.GetChassisBody(), 5.f, offset_pose, gps_reference, noise_none);
    gps->PushFilter(chrono_types::make_shared<ChFilterGPSAccess>());
    sensor_manager->AddSensor(gps);
    sensor_manager->Update();

    auto acc_rate = acc->GetUpdateRate() / 2;
    auto acc_topic_name = "/artcar_1/output/accelerometer/data";
    auto acc_handler = chrono_types::make_shared<ChROSAccelerometerHandler>(acc_rate, acc, acc_topic_name);
    ros_manager_1->RegisterHandler(acc_handler);

    // Create the publisher for the gyroscope
    auto gyro_topic_name = "/artcar_1/output/gyroscope/data";
    auto gyro_handler = chrono_types::make_shared<ChROSGyroscopeHandler>(gyro, gyro_topic_name);
    ros_manager_1->RegisterHandler(gyro_handler);

    // Create the publisher for the magnetometer
    auto mag_topic_name = "/artcar_1/output/magnetometer/data";
    auto mag_handler = chrono_types::make_shared<ChROSMagnetometerHandler>(mag, mag_topic_name);
    ros_manager_1->RegisterHandler(mag_handler);

    // Create the publisher for the GPS
    auto gps_topic_name = "/artcar_1/output/gps/data";
    auto gps_handler = chrono_types::make_shared<ChROSGPSHandler>(gps, gps_topic_name);
    ros_manager_1->RegisterHandler(gps_handler);

    // Finally, initialize the ros manager
    ros_manager_1->Initialize();

    // Create and initialize the second vehicle (ARTcar)
    ARTcar artcar_2(&sys);
    artcar_2.SetInitPosition(ChCoordsys<>(ChVector3d(-3.5, 0, 0.2), QUNIT));    
    artcar_2.Initialize();
    artcar_2.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    artcar_2.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    artcar_2.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    artcar_2.SetWheelVisualizationType(VisualizationType::NONE);
    artcar_2.SetTireVisualizationType(VisualizationType::PRIMITIVES);

    // Create the basic driver for the first vehicle
    auto driver_2 = std::make_shared<ChDriver>(artcar_2.GetVehicle());
    // Create ROS manager for vehicle one
    auto ros_manager_2 = chrono_types::make_shared<ChROSManager>("artcar_2");
    // Create a subscriber to the driver inputs
    auto driver_inputs_handler_2 =
        chrono_types::make_shared<ChROSDriverInputsHandler>(driver_inputs_rate, driver_2, driver_inputs_topic_name);
    ros_manager_2->RegisterHandler(driver_inputs_handler_2);
    // Create a publisher for the vehicle state
    auto vehicle_state_handler_2 = chrono_types::make_shared<ChROSBodyHandler>(
        vehicle_state_rate, artcar_2.GetChassisBody(), vehicle_state_topic_name);
    ros_manager_2->RegisterHandler(vehicle_state_handler_2);
    // Finally, initialize the ros manager

    ChVector3d gps_reference2(-89.400, 43.070, 260.0);
    auto acc2 = chrono_types::make_shared<ChAccelerometerSensor>(artcar_2.GetChassisBody(), 100.f, offset_pose, noise_none);
    acc2->PushFilter(chrono_types::make_shared<ChFilterAccelAccess>());
    sensor_manager->AddSensor(acc2);

    auto gyro2 = chrono_types::make_shared<ChGyroscopeSensor>(artcar_2.GetChassisBody(), 100.f, offset_pose, noise_none);
    gyro2->PushFilter(chrono_types::make_shared<ChFilterGyroAccess>());
    sensor_manager->AddSensor(gyro2);

    auto mag2 =
        chrono_types::make_shared<ChMagnetometerSensor>(artcar_2.GetChassisBody(), 100.f, offset_pose, noise_none, gps_reference);
    mag2->PushFilter(chrono_types::make_shared<ChFilterMagnetAccess>());
    sensor_manager->AddSensor(mag2);

    // add a GPS sensor
    auto gps2 = chrono_types::make_shared<ChGPSSensor>(artcar_2.GetChassisBody(), 5.f, offset_pose, gps_reference, noise_none);
    gps2->PushFilter(chrono_types::make_shared<ChFilterGPSAccess>());
    sensor_manager->AddSensor(gps2);
    sensor_manager->Update();

    auto acc_rate2 = acc->GetUpdateRate() / 2;
    auto acc_topic_name2 = "/artcar_2/output/accelerometer/data";
    auto acc_handler2 = chrono_types::make_shared<ChROSAccelerometerHandler>(acc_rate2, acc2, acc_topic_name2);
    ros_manager_2->RegisterHandler(acc_handler2);

    // Create the publisher for the gyroscope
    auto gyro_topic_name2 = "/artcar_2/output/gyroscope/data";
    auto gyro_handler2 = chrono_types::make_shared<ChROSGyroscopeHandler>(gyro2, gyro_topic_name2);
    ros_manager_2->RegisterHandler(gyro_handler2);

    // Create the publisher for the magnetometer
    auto mag_topic_name2 = "/artcar_2/output/magnetometer/data";
    auto mag_handler2 = chrono_types::make_shared<ChROSMagnetometerHandler>(mag2, mag_topic_name2);
    ros_manager_2->RegisterHandler(mag_handler2);

    // Create the publisher for the GPS
    auto gps_topic_name2 = "/artcar_2/output/gps/data";
    auto gps_handler2 = chrono_types::make_shared<ChROSGPSHandler>(gps2, gps_topic_name2);
    ros_manager_2->RegisterHandler(gps_handler2);

    ros_manager_2->Initialize();

// Create the vehicle run-time visualization interface and the interactive driver
#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    std::shared_ptr<ChVehicleVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
            vis_irr->SetWindowTitle("Two cars demo");
            vis_irr->SetChaseCamera(ChVector3d(0.0, 0.0, .75), 6.0, 0.5);
            vis_irr->SetChaseCameraState(utils::ChChaseCamera::Track);
            vis_irr->SetChaseCameraPosition(ChVector3d(-15, 0, 2.0));
            vis_irr->Initialize();
            vis_irr->AddSkyBox();
            vis_irr->AddLogo();
            vis_irr->AddLightDirectional();
            vis_irr->AttachVehicle(&artcar_1.GetVehicle());

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
            vis_vsg->SetWindowTitle("Two cars demo");
            vis_vsg->SetWindowSize(ChVector2i(1200, 800));
            vis_vsg->SetWindowPosition(ChVector2i(100, 300));
            vis_vsg->SetChaseCamera(ChVector3d(0.0, 0.0, .75), 6.0, 0.5);
            vis_vsg->SetChaseCameraState(utils::ChChaseCamera::Track);
            vis_vsg->SetChaseCameraPosition(ChVector3d(-15, 0, 2.0));
            vis_vsg->AttachVehicle(&artcar_1.GetVehicle());
            vis_vsg->SetUseSkyBox(true);
            vis_vsg->SetCameraAngleDeg(40);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->SetShadows(true);
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // ---------------
    // Simulation loop
    // ---------------

    artcar_1.GetVehicle().EnableRealtime(true);
    artcar_2.GetVehicle().EnableRealtime(true);

    while (vis->Run()) {
        double time = sys.GetChTime();

        // Render scene
        vis->BeginScene();
        vis->Render();

        // Driver inputs
        DriverInputs driver_inputs_1 = driver_1->GetInputs();
        DriverInputs driver_inputs_2 = driver_2->GetInputs();

        // Update modules (process inputs from other modules)
        driver_1->Synchronize(time);
        driver_2->Synchronize(time);
        artcar_1.Synchronize(time, driver_inputs_1, terrain);
        artcar_2.Synchronize(time, driver_inputs_2, terrain);
        terrain.Synchronize(time);
        vis->Synchronize(time, driver_inputs_1);

        // Advance simulation for one timestep for all modules.
        driver_1->Advance(step_size);
        driver_2->Advance(step_size);
        artcar_1.Advance(step_size);
        artcar_2.Advance(step_size);
        terrain.Advance(step_size);
        vis->Advance(step_size);


        // Advance state of entire system (containing both vehicles)
        sys.DoStepDynamics(step_size);
            
        sensor_manager->Update();
        // Update ROS managers
        if (!ros_manager_1->Update(time, step_size) || !ros_manager_2->Update(time, step_size))
            break;

        vis->EndScene();
    }

    return 0;
}
