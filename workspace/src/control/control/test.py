import numpy as np
import pydof18 as d18
import copy

class YourVehicleController:
    def __init__(self):
        self.init_vehicle()



    def init_vehicle(self):

        vehParamsJsonPath = "/home/ishaan/low-fidelity-dynamic-models/wheeled_vehicle_models/18dof/data/json/ART/vehicle.json"
        tireParamsJsonPath = "/home/ishaan/low-fidelity-dynamic-models/wheeled_vehicle_models/18dof/data/json/ART/tmeasy.json"
        driver_file = "/home/ishaan/low-fidelity-dynamic-models/wheeled_vehicle_models/18dof/data/input/acc.txt"
        solver = d18.d18SolverHalfImplicit()
        solver.Construct(vehParamsJsonPath, tireParamsJsonPath, driver_file)

        # Set time step
        solver.SetTimeStep(1e-3)

        # Initialize solver (set initial conditions)
        veh_st = d18.VehicleState()
        tirelf_st = d18.TMeasyState()
        tirerf_st = d18.TMeasyState()
        tirelr_st = d18.TMeasyState()
        tirerr_st = d18.TMeasyState()
        solver.Initialize(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st)

        self.solver = solver
        self.throttle = 0
        self.steering = 0
        self.t = 0
        
    # def dof18_model(self, t, throttle, steering):
    #     required_time = t + 0.1    
    #     new_time = t
    #     while(new_time < required_time):
    #         new_time = self.solver.IntegrateStep(t, throttle, steering, 0)
    #         t = new_time

    #     return self.solver


    def dof18_model(self,solverOld, t, throttle, steering):
        
        required_time = t + 0.1

        # Create a new solver object
        solverNew = d18.d18SolverHalfImplicit()

        # copy over solver parameters that were there at the start of the iterations (ideally also the same as the initial parameters)
        solverNew.m_veh_param = d18.VehicleParam(solverOld.m_veh_param)
        solverNew.m_tire_param = d18.TMeasyParam(solverOld.m_tire_param)

        # Set the simulation time step
        solverNew.SetTimeStep(1e-3)

        # Copy over the solver states that were there at the start of the iterations
        solverNew.m_veh_st = d18.VehicleState(solverOld.m_veh_state)
        solverNew.m_tirelf_st = d18.TMeasyState(solverOld.m_tirelf_state)
        solverNew.m_tirerf_st = d18.TMeasyState(solverOld.m_tirerf_state)
        solverNew.m_tirelr_st = d18.TMeasyState(solverOld.m_tirelr_state)
        solverNew.m_tirerr_st = d18.TMeasyState(solverOld.m_tirerr_state)
        
        
        i = 0
        # Integrate till the required time
        while t < required_time:
            # Integrate till the next time step
            new_time = solverNew.IntegrateStep(t, throttle, steering, 0)
            t = new_time  # new_time is where the solver is right now
            
            
            
        # Once we have intgrated till the required time, we can return the solver object
        self.t = t
        return solverNew


    def pid_control(self,error, error_sum, error_diff, error_prev):
        # Set PID parameters
        Kp = 1
        Ki = 1
        Kd = 1

        # do some PID

        error_sum += error
        error_diff = error - error_prev
        error_prev = error

        throttle = Kp*error + Ki*error_sum + Kd*error_diff

        throttle = np.clip(throttle, 0, 1)

        return throttle


    def main(self):
            throttle = self.throttle
            max_iterations = 10
            iteration = 0

    
            error_sum = 0
            error_diff = 0
            error_prev = 0

            
            steering = self.steering
            while iteration < max_iterations:

                # Call the model with the current throttle and steering
                solverUpdated = self.dof18_model(self.solver, self.t, throttle, steering)

                # Extract the velocity from the solver
                dof18_vel = solverUpdated.m_veh_st._u

                # Calculate the error (for now just set refernce to 0.2)
                error = 0.2 - dof18_vel

                # Call the PID controller
                throttle = self.pid_control(error, error_sum, error_diff, error_prev)

                iteration += 1

            self.throttle = throttle
            print(self.throttle)
            # Finally update the global solver states
            self.solver.m_veh_st = d18.VehicleState(solverUpdated.m_veh_state)
            self.solver.m_tirelf_st = d18.TMeasyState(solverUpdated.m_tirelf_state)
            self.solver.m_tirerf_st = d18.TMeasyState(solverUpdated.m_tirerf_state)
            self.solver.m_tirelr_st = d18.TMeasyState(solverUpdated.m_tirelr_state)
            self.solver.m_tirerr_st = d18.TMeasyState(solverUpdated.m_tirerr_state)


if __name__ == "__main__":
    controller = YourVehicleController()
    controller.main()
            

    