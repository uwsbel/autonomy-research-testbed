#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#include "chrono_models/vehicle/artcar/ARTcar.h"
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include <vector>
#include <fstream>
#include <algorithm>
#include <chrono>
#include <thread>

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::artcar;
using namespace chrono::irrlicht;  // Namespace for Irrlicht-related classes

double ComputeSlipRatio(double vehicle_speed, double wheel_speed, double wheel_radius) {
    double slip_ratio = 0.0;

    if (vehicle_speed < wheel_radius * wheel_speed) {
        slip_ratio = 1.0 - vehicle_speed / (wheel_radius * wheel_speed);
    } else if (vehicle_speed > wheel_radius * wheel_speed) {
        slip_ratio = (wheel_radius * wheel_speed) / vehicle_speed - 1.0;
    } else {
        slip_ratio = 0.0;
    }

    return std::clamp(slip_ratio, -1.0, 1.0);
}

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    chrono::SetChronoDataPath("/opt/chrono/share/chrono/data/");
    ChSystemSMC sys; // Using ChSystemSMC for SCM terrain simulation
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    auto collsys = chrono_types::make_shared<ChCollisionSystemBullet>();
    sys.SetCollisionSystem(collsys);
            
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    sys.GetSolver()->AsIterative()->SetMaxIterations(150);
    sys.SetMaxPenetrationRecoverySpeed(4.0);
    sys.SetNumThreads(std::min(8, ChOMP::GetNumProcs()));

    TireModelType tire_model = TireModelType::RIGID;

    auto artcar = chrono_types::make_shared<ARTcar>(&sys);
    artcar->SetInitPosition(ChCoordsys<>(ChVector3d(0, 0, 0.5), QUNIT));
    artcar->SetStallTorque(0.001f);
    artcar->SetMaxMotorVoltageRatio(0.3f);
    artcar->SetTireRollingResistance(0.0015f);
    artcar->SetTireType(tire_model);
    artcar->SetContactMethod(ChContactMethod::NSC);
    // auto wheel = artcar->GetVehicle().GetWheel(0,LEFT)->GetSpindle();


    artcar->Initialize();

    // Set visualization types for various components
    artcar->SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    artcar->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    artcar->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    artcar->SetWheelVisualizationType(VisualizationType::PRIMITIVES);
    artcar->SetTireVisualizationType(VisualizationType::PRIMITIVES);

    auto driver = std::make_shared<ChDriver>(artcar->GetVehicle());

    

    // Setup the SCM terrain
    vehicle::SCMTerrain terrain(&sys);
    terrain.SetPlane(ChCoordsys<>(ChVector3d(0, 0, -0.5), QUNIT));  // Adjust plane height for your use case
    terrain.Initialize(5, 1, 0.0125);  // 100x100 terrain with a 5cm grid resolution

    // Set the soil terramechanical parameters
    terrain.SetSoilParameters(0.82e6,   // Bekker Kphi
                              0.14e4,   // Bekker Kc
                              1.0,      // Bekker n exponent
                              0.017e4,  // Mohr cohesive limit (Pa)
                              35,       // Mohr friction limit (degrees)
                              1.78e-2,  // Janosi shear coefficient (m)
                              2e8,      // Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
                              3e4       // Damping (Pa s/m), proportional to negative vertical speed (optional)
    );
    
    // Create the Irrlicht application for visualization
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowSize(1200, 800);
    vis->SetWindowTitle("Multiple Cars Demo with SCM Terrain");
    vis->SetChaseCamera(ChVector3d(0.0, 0.0, .5), 1.0, 0.5);
    vis->SetChaseCameraState(utils::ChChaseCamera::Follow);
    vis->SetChaseCameraPosition(ChVector3d(-1.5, 0, 1.0));
    vis->Initialize();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AddTypicalLights();
    vis->AttachVehicle(&artcar->GetVehicle());
    vis->EnableShadows();

    double step_size = 1e-3;
    double simulation_time = 10.0;
    double time = 0.0;

    std::vector<double> throttle_inputs = {1.0,1.0,1.0,1.0,1.0};
    // std::vector<double> throttle_inputs = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    std::vector<double> slip_ratio_data;
    std::vector<double> drawbar_pull_data;

    // Settling phase for 3 seconds
    double settling_time = 10.0;
    auto last_vis_update_time = std::chrono::steady_clock::now();
    double orig_time = sys.GetChTime();
    while (time - orig_time < settling_time) {
        time = sys.GetChTime();

        // Update driver inputs and synchronize the driver
        DriverInputs driver_inputs = driver->GetInputs();
        driver->Synchronize(time);

        // Synchronize other systems
        artcar->Synchronize(time, driver_inputs, terrain);
        terrain.Synchronize(time);

        // Advance the simulation for all components
        driver->Advance(step_size);
        artcar->Advance(step_size);
        terrain.Advance(step_size);

        sys.DoStepDynamics(step_size);

        auto current_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = current_time - last_vis_update_time;
        if (elapsed.count() >= (1.0 / 15.0)) {  // 15 FPS visualization rate
            vis->BeginScene();
            vis->Render();
            vis->Advance(step_size);
            vis->Synchronize(time, driver->GetInputs());
            vis->EndScene();
            last_vis_update_time = current_time;
        }
    }

    for (double throttle : throttle_inputs) {
        time = 0.0;
        driver->SetThrottle(throttle);
        driver->SetBraking(0.0);
        driver->SetSteering(0.0);

        std::cout << "Simulating with Throttle: " << throttle << std::endl;

        // Reset time to 0 for data collection after settling
        time = 0.0;

        // Main data collection loop after settling phase
        double accumulated_force = 0.0;
        double time_orig = sys.GetChTime();
        last_vis_update_time = std::chrono::steady_clock::now();

        while (time < time_orig + simulation_time) {
            time = sys.GetChTime();

            // Update driver inputs and synchronize the driver
            DriverInputs driver_inputs = driver->GetInputs();
            driver->Synchronize(time);

            // Synchronize other systems
            artcar->Synchronize(time, driver_inputs, terrain);
            terrain.Synchronize(time);

            // Advance the simulation for all components
            driver->Advance(step_size);
            artcar->Advance(step_size);
            terrain.Advance(step_size);

            // Calculate vehicle dynamics
            double vehicle_speed = artcar->GetVehicle().GetChassisBody()->GetPosDt().Length();

            // Get the slip ratio for the left tire of the first axle only
            double slip_ratio_left = std::clamp(artcar->GetVehicle().GetTire(0, LEFT)->GetLongitudinalSlip(), -1.0, 1.0);

            // Calculate the drawbar pull for the left tire of the first axle
            auto frame = artcar->GetVehicle().GetTire(0, LEFT)->GetTransform().GetCoordsys();
            // double left_tire_force = artcar->GetVehicle().GetTire(0, LEFT)->ReportTireForceLocal(&terrain, frame).force.x();
            auto wheel = artcar->GetVehicle().GetWheel(0,LEFT)->GetSpindle();
            ChVector3d force;
            ChVector3d torque;

            terrain.GetContactForceBody(wheel,force,torque);

            double left_tire_force = force.y() + force.x();
            // Accumulate the drawbar pull force
            accumulated_force += left_tire_force;

            // Store slip ratio and drawbar pull data
            slip_ratio_data.push_back(slip_ratio_left);
            drawbar_pull_data.push_back(left_tire_force);

            sys.DoStepDynamics(step_size);

            auto current_time = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed = current_time - last_vis_update_time;
            if (elapsed.count() >= (1.0 / 15.0)) {  // 15 FPS visualization rate
                vis->BeginScene();
                vis->Render();
                vis->Advance(step_size);
                vis->Synchronize(time, driver->GetInputs());
                vis->EndScene();
                last_vis_update_time = current_time;
            }
        }

        std::cout << "Accumulated drawbar pull for throttle " << throttle << ": " << accumulated_force << std::endl;
    }

    std::ofstream csv_file("slip_ratio_vs_drawbar_pull.csv");
    csv_file << "Slip Ratio,Drawbar Pull\n";
    for (size_t i = 0; i < slip_ratio_data.size(); ++i) {
        csv_file << slip_ratio_data[i] << "," << drawbar_pull_data[i] << "\n";
    }
    csv_file.close();

    std::cout << "Slip ratio vs drawbar pull data has been saved to slip_ratio_vs_drawbar_pull.csv" << std::endl;

    return 0;
}