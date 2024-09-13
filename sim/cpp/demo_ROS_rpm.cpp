#include "chrono/physics/ChSystemNSC.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_models/vehicle/artcar/ARTcar.h"

#include <vector>
#include <fstream>  // Include fstream for file handling

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::artcar;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // --------------
    // Create systems
    // --------------
    chrono::SetChronoDataPath("/opt/chrono/share/chrono/data/");
    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    sys.GetSolver()->AsIterative()->SetMaxIterations(150);
    sys.SetMaxPenetrationRecoverySpeed(4.0);

    // Create a single ARTcar and suspend it in midair
    auto artcar = chrono_types::make_shared<ARTcar>(&sys);
    artcar->SetInitPosition(ChCoordsys<>(ChVector3d(0, 0, 5), QUNIT));  // Suspend in midair at 5 meters height
    artcar->SetStallTorque(0.0195);
    artcar->SetMaxMotorVoltageRatio(0.245f);
    artcar->Initialize();

    // Disable visualizations for simplicity
    artcar->SetChassisVisualizationType(VisualizationType::NONE);
    artcar->SetSuspensionVisualizationType(VisualizationType::NONE);
    artcar->SetSteeringVisualizationType(VisualizationType::NONE);
    artcar->SetWheelVisualizationType(VisualizationType::NONE);
    artcar->SetTireVisualizationType(VisualizationType::NONE);

    // Create a driver to control the vehicle
    auto driver = std::make_shared<ChDriver>(artcar->GetVehicle());

    // Create a dummy terrain (not used, but required by the Synchronize function)
    RigidTerrain terrain(&sys);
    auto patch_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    patch_mat->SetFriction(0.0f);
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, 100, 100);
    terrain.Initialize();

    // Simulation parameters
    double step_size = 2e-3;
    double simulation_time = 20.0;  // 30 seconds of simulation
    double time = 0.0;

    // Prepare the control inputs for different scenarios
    struct ControlInput {
        double throttle;
        double brake;
        double steering;
    };

    std::vector<ControlInput> control_inputs = {
        {0.1, 0.0, 0.0},  // Light throttle, no brake, no steering
        {0.2, 0.0, 0.0},  // Light throttle, no brake, no steering
        {0.3, 0.0, 0.0},  // Light throttle, no brake, no steering
        {0.4, 0.0, 0.0},  // Light throttle, no brake, no steering
        {0.5, 0.0, 0.0},  // Light throttle, no brake, no steering
        {0.6, 0.0, 0.0},  // Light throttle, no brake, no steering
        {0.7, 0.0, 0.0},  // Medium throttle, no brake, no steering
        {0.8, 0.0, 0.0},  // Medium throttle, no brake, no steering
        {0.9, 0.0, 0.0},  // Medium throttle, no brake, no steering
        {1.0, 0.0, 0.0},  // Full throttle, no brake, no steering
    };

    // Storage for throttle and RPM data
    std::vector<double> throttle_data;
    std::vector<double> rpm_data_left_front;
    std::vector<double> rpm_data_right_front;
    std::vector<double> rpm_data_left_rear;
    std::vector<double> rpm_data_right_rear;

    // ---------------
    // Simulation loop
    // ---------------
    for (const auto& input : control_inputs) {
        time = 0.0;
        driver->SetThrottle(input.throttle);
        driver->SetBraking(input.brake);
        driver->SetSteering(input.steering);

        std::cout << "Simulating with Throttle: " << input.throttle 
                << ", Brake: " << input.brake 
                << ", Steering: " << input.steering << std::endl;

        // Variables for equilibrium detection
        const int equilibrium_steps = 100;  // Number of consecutive steps to check for equilibrium
        const double rpm_threshold = 1.0;   // Threshold for change in RPM to consider equilibrium
        int stable_step_count = 0;          // Count of consecutive steps where RPM is stable
        const double min_time_before_equilibrium_check = 30.0;  // Minimum time to run before checking for equilibrium

        double prev_rpm_left_front = 0.0;
        double prev_rpm_right_front = 0.0;
        double prev_rpm_left_rear = 0.0;
        double prev_rpm_right_rear = 0.0;

        // Acceleration phase
        while (time < simulation_time) {
            // Synchronize the driver
            DriverInputs driver_inputs = driver->GetInputs();
            driver->Synchronize(time);
            artcar->Synchronize(time, driver_inputs, terrain);

            // Advance simulation for one timestep
            driver->Advance(step_size);
            artcar->Advance(step_size);
            sys.DoStepDynamics(step_size);

            // Calculate the RPM for each wheel
            double rpm_left_front = -artcar->GetVehicle().GetSpindleOmega(0, LEFT) * 9.5493;
            double rpm_right_front = -artcar->GetVehicle().GetSpindleOmega(0, RIGHT) * 9.5493;
            double rpm_left_rear = -artcar->GetVehicle().GetSpindleOmega(1, LEFT) * 9.5493;
            double rpm_right_rear = -artcar->GetVehicle().GetSpindleOmega(1, RIGHT) * 9.5493;

            // Store the throttle and RPM data
            throttle_data.push_back(driver_inputs.m_throttle);
            rpm_data_left_front.push_back(rpm_left_front);
            rpm_data_right_front.push_back(rpm_right_front);
            rpm_data_left_rear.push_back(rpm_left_rear);
            rpm_data_right_rear.push_back(rpm_right_rear);

            // Only start checking for equilibrium after 30 seconds
            if (time > min_time_before_equilibrium_check) {
                // Check if RPM change is below threshold
                if (std::abs(rpm_left_front - prev_rpm_left_front) < rpm_threshold &&
                    std::abs(rpm_right_front - prev_rpm_right_front) < rpm_threshold &&
                    std::abs(rpm_left_rear - prev_rpm_left_rear) < rpm_threshold &&
                    std::abs(rpm_right_rear - prev_rpm_right_rear) < rpm_threshold) {
                    // Increment stable step count if RPM changes are small
                    stable_step_count++;
                } else {
                    // Reset stable step count if RPM changes are significant
                    stable_step_count = 0;
                }

                // Break if equilibrium is reached
                if (stable_step_count >= equilibrium_steps) {
                    std::cout << "Equilibrium reached for throttle: " << input.throttle << std::endl;
                    break;  // Exit acceleration phase
                }
            }

            // Update previous RPM values
            prev_rpm_left_front = rpm_left_front;
            prev_rpm_right_front = rpm_right_front;
            prev_rpm_left_rear = rpm_left_rear;
            prev_rpm_right_rear = rpm_right_rear;

            // Advance time
            time += step_size;
        }

        // Braking phase
        std::cout << "Braking to stop the vehicle..." << std::endl;
        double brake_time = 0.0;
        double braking_step_size = step_size;
        while (true) {
            // Apply maximum braking force
            driver->SetThrottle(0.0);
            driver->SetBraking(1.0);

            // Synchronize the driver
            DriverInputs driver_inputs = driver->GetInputs();
            driver->Synchronize(brake_time);
            artcar->Synchronize(brake_time, driver_inputs, terrain);

            // Advance simulation for one timestep
            driver->Advance(braking_step_size);
            artcar->Advance(braking_step_size);
            sys.DoStepDynamics(braking_step_size);

            // Calculate the RPM for each wheel
            double rpm_left_front = -artcar->GetVehicle().GetSpindleOmega(0, LEFT) * 9.5493;
            double rpm_right_front = -artcar->GetVehicle().GetSpindleOmega(0, RIGHT) * 9.5493;
            double rpm_left_rear = -artcar->GetVehicle().GetSpindleOmega(1, LEFT) * 9.5493;
            double rpm_right_rear = -artcar->GetVehicle().GetSpindleOmega(1, RIGHT) * 9.5493;

            // Check if all wheels have stopped (RPM is close to zero)
            if (std::abs(rpm_left_front) < 5e-2 && std::abs(rpm_right_front) < 5e-2 &&
                std::abs(rpm_left_rear) < 5e-2 && std::abs(rpm_right_rear) < 5e-2) {
                std::cout << "Vehicle has come to a stop." << std::endl;
                break;  // Exit the braking loop
            }

            // Advance brake time
            brake_time += braking_step_size;
        }
    }



    // Write the data to a CSV file
    std::ofstream csv_file("wheel_rpm_vs_throttle.csv");
    csv_file << "Throttle,Left Front RPM,Right Front RPM,Left Rear RPM,Right Rear RPM\n";
    for (size_t i = 0; i < throttle_data.size(); ++i) {
        csv_file << throttle_data[i] << ","
                 << rpm_data_left_front[i] << ","
                 << rpm_data_right_front[i] << ","
                 << rpm_data_left_rear[i] << ","
                 << rpm_data_right_rear[i] << "\n";
    }
    csv_file.close();

    std::cout << "RPM data with respect to throttle has been saved to wheel_rpm_vs_throttle.csv" << std::endl;

    return 0;
}
