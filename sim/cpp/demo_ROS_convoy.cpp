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
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <signal.h>

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::artcar;
using namespace chrono::ros;

// =============================================================================

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::IRRLICHT;

// Simulation step sizes
double step_size = 1e-3;
using namespace chrono::sensor;

// =============================================================================
int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // --------------
    // Create systems
    // --------------
    chrono::SetChronoDataPath("/opt/chrono/share/chrono/data/");
    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    sys.GetSolver()->AsIterative()->SetMaxIterations(150);
    sys.SetMaxPenetrationRecoverySpeed(4.0);

    // Create the terrain
    RigidTerrain terrain(&sys);
    auto patch_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    patch_mat->SetFriction(0.7f);
    patch_mat->SetRestitution(0.01f);
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, 200, 100);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/cubetexture_pinkwhite.png"), 200, 200);
    terrain.Initialize();

    // Number of vehicles
    int num_vehicles = 4;
    std::vector<std::shared_ptr<ARTcar>> artcars(num_vehicles);
    std::vector<std::shared_ptr<ChDriver>> drivers(num_vehicles);
    std::vector<std::shared_ptr<ChROSManager>> ros_managers(num_vehicles);

    // Initialize vehicles
    for (int i = 0; i < num_vehicles; ++i) {
        artcars[i] = chrono_types::make_shared<ARTcar>(&sys);
        artcars[i]->SetInitPosition(ChCoordsys<>(ChVector3d(-3.5 * i, 0, 0.2), QUNIT));
        artcars[i]->Initialize();
        artcars[i]->SetChassisVisualizationType(VisualizationType::PRIMITIVES);
        artcars[i]->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
        artcars[i]->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
        artcars[i]->SetWheelVisualizationType(VisualizationType::NONE);
        artcars[i]->SetTireVisualizationType(VisualizationType::PRIMITIVES);
        artcars[i]->SetTireRollingResistance(0.05f);
        artcars[i]->SetStallTorque(0.09f);
        // artcars[i]->SetMaxMotorVoltageRatio(0.3f);

        drivers[i] = std::make_shared<ChDriver>(artcars[i]->GetVehicle());
        
        ros_managers[i] = chrono_types::make_shared<ChROSManager>("artcar_" + std::to_string(i + 1));
        auto driver_inputs_handler = chrono_types::make_shared<ChROSDriverInputsHandler>(25, drivers[i], "~/input/driver_inputs");
        ros_managers[i]->RegisterHandler(driver_inputs_handler);
        
        auto vehicle_state_handler = chrono_types::make_shared<ChROSBodyHandler>(25, artcars[i]->GetChassisBody(), "~/output/vehicle/state");
        ros_managers[i]->RegisterHandler(vehicle_state_handler);
        
        ros_managers[i]->Initialize();
    }

    auto noise_none = chrono_types::make_shared<ChNoiseNone>();
    chrono::ChFrame<double> offset_pose({0, 0, 2}, QuatFromAngleAxis(0, {0, 0, 1}));
    auto sensor_manager = chrono_types::make_shared<ChSensorManager>(&sys);
    sensor_manager->scene->AddPointLight({100, 100, 100}, {2, 2, 2}, 500);
    sensor_manager->scene->SetAmbientLight({0.1f, 0.1f, 0.1f});

    for (int i = 0; i < num_vehicles; ++i) {
        ChVector3d gps_reference(-89.41161, 43.07203, 260.0);

        auto acc = chrono_types::make_shared<ChAccelerometerSensor>(artcars[i]->GetChassisBody(), 100.f, offset_pose, noise_none);
        acc->PushFilter(chrono_types::make_shared<ChFilterAccelAccess>());
        sensor_manager->AddSensor(acc);
        
        auto gyro = chrono_types::make_shared<ChGyroscopeSensor>(artcars[i]->GetChassisBody(), 100.f, offset_pose, noise_none);
        gyro->PushFilter(chrono_types::make_shared<ChFilterGyroAccess>());
        sensor_manager->AddSensor(gyro);
        
        auto mag = chrono_types::make_shared<ChMagnetometerSensor>(artcars[i]->GetChassisBody(), 100.f, offset_pose, noise_none, gps_reference);
        mag->PushFilter(chrono_types::make_shared<ChFilterMagnetAccess>());
        sensor_manager->AddSensor(mag);
        
        auto gps = chrono_types::make_shared<ChGPSSensor>(artcars[i]->GetChassisBody(), 80.f, offset_pose, gps_reference, noise_none);
        gps->PushFilter(chrono_types::make_shared<ChFilterGPSAccess>());
        sensor_manager->AddSensor(gps);
        sensor_manager->Update();

        auto acc_rate = acc->GetUpdateRate() / 2;
        auto acc_topic_name = "/artcar_" + std::to_string(i + 1) + "/output/accelerometer/data";
        auto acc_handler = chrono_types::make_shared<ChROSAccelerometerHandler>(acc_rate, acc, acc_topic_name);
        ros_managers[i]->RegisterHandler(acc_handler);
        
        auto gyro_topic_name = "/artcar_" + std::to_string(i + 1) + "/output/gyroscope/data";
        auto gyro_handler = chrono_types::make_shared<ChROSGyroscopeHandler>(gyro, gyro_topic_name);
        ros_managers[i]->RegisterHandler(gyro_handler);
        
        auto mag_topic_name = "/artcar_" + std::to_string(i + 1) + "/output/magnetometer/data";
        auto mag_handler = chrono_types::make_shared<ChROSMagnetometerHandler>(mag, mag_topic_name);
        ros_managers[i]->RegisterHandler(mag_handler);
        
        auto gps_topic_name = "/artcar_" + std::to_string(i + 1) + "/output/gps/data";
        auto gps_handler = chrono_types::make_shared<ChROSGPSHandler>(gps, gps_topic_name);
        ros_managers[i]->RegisterHandler(gps_handler);

        ros_managers[i]->Initialize();
    }

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
            vis_irr->SetWindowTitle("Multiple cars demo");
            vis_irr->SetChaseCamera(ChVector3d(0.0, 0.0, .75), 3.0, 0.5);
            vis_irr->SetChaseCameraState(utils::ChChaseCamera::Follow);
            vis_irr->SetChaseCameraPosition(ChVector3d(-15, 0, 2.0));
            vis_irr->Initialize();
            vis_irr->AddSkyBox();
            vis_irr->AddLogo();
            vis_irr->AddLightDirectional();
            vis_irr->AttachVehicle(&artcars[2]->GetVehicle());

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
            vis_vsg->SetWindowTitle("Multiple cars demo");
            vis_vsg->SetWindowSize(ChVector2i(1200, 800));
            vis_vsg->SetWindowPosition(ChVector2i(100, 300));
            vis_vsg->SetChaseCamera(ChVector3d(0.0, 0.0, .75), 6.0, 0.5);
            vis_vsg->SetChaseCameraState(utils::ChChaseCamera::Track);
            vis_vsg->SetChaseCameraPosition(ChVector3d(-15, 0, 2.0));
            vis_vsg->AttachVehicle(&artcars[0]->GetVehicle());
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
    for (auto& artcar : artcars) {
        artcar->GetVehicle().EnableRealtime(true);
    }

    while (vis->Run()) {
        double time = sys.GetChTime();

        // Render scene
        vis->BeginScene();
        vis->Render();

        // Driver inputs
        for (int i = 0; i < num_vehicles; ++i) {
            DriverInputs driver_inputs = drivers[i]->GetInputs();
            drivers[i]->Synchronize(time);
            artcars[i]->Synchronize(time, driver_inputs, terrain);
        }

        terrain.Synchronize(time);
        vis->Synchronize(time, drivers[0]->GetInputs());

        // Advance simulation for one timestep for all modules.
        for (int i = 0; i < num_vehicles; ++i) {
            drivers[i]->Advance(step_size);
            artcars[i]->Advance(step_size);
        }

        terrain.Advance(step_size);
        vis->Advance(step_size);

        // Advance state of entire system
        sys.DoStepDynamics(step_size);
        sensor_manager->Update();

        // Update ROS managers
        bool ros_update_success = true;
        for (auto& ros_manager : ros_managers) {
            ros_update_success &= ros_manager->Update(time, step_size);
        }
        if (!ros_update_success)
            break;

        vis->EndScene();
    }

    return 0;
}
