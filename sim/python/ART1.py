"""Wrapper around the chrono.vehicle.RCCar class which matches the ART-1 platform.

It derives contains the veh.RCCar and all the sensors associated with the physical
platform.
"""

# Standard Imports
from pathlib import Path

# Chrono Imports
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens


class ART1(veh.RCCar):
    def __init__(self, init_loc: chrono.ChVectorD, init_rot: chrono.ChQuaternionD):
        self.veh = veh.RCCar()
        self.veh.SetContactMethod(chrono.ChContactMethod_NSC)
        self.veh.SetChassisCollisionType(veh.CollisionType_NONE)
        self.veh.SetChassisFixed(False)
        self.veh.SetTireType(veh.TireModelType_TMEASY)
        self.veh.SetInitPosition(chrono.ChCoordsysD(init_loc, init_rot))
        self.veh.SetTireStepSize(1e-3)
        self.veh.Initialize()

        chassis_body = self.veh.GetChassisBody()

        self.driver = veh.ChDriver(self.veh.GetVehicle())

        self._sensors = []
        sensor_path = Path(chrono.GetChronoDataFile("autonomy/art-1/sensors/"))

        camera_pos = chrono.ChVectorD(0.204, 0, 0.10018)
        camera_rot = chrono.Q_from_AngAxis(0.1, chrono.ChVectorD(0, 1, 0))
        camera_pose = chrono.ChFrameD(camera_pos, camera_rot)
        self.cam = sens.CastToChCameraSensor(
            sens.Sensor.CreateFromJSON(
                str(sensor_path / "camera.json"), chassis_body, camera_pose
            )
        )

        offset_pose = chrono.ChFrameD()
        self.acc = sens.CastToChAccelerometerSensor(
            sens.Sensor.CreateFromJSON(
                str(sensor_path / "accelerometer.json"), chassis_body, offset_pose
            )
        )
        self.gyro = sens.CastToChGyroscopeSensor(
            sens.Sensor.CreateFromJSON(
                str(sensor_path / "gyroscope.json"), chassis_body, offset_pose
            )
        )
        self.mag = sens.CastToChMagnetometerSensor(
            sens.Sensor.CreateFromJSON(
                str(sensor_path / "magnetometer.json"), chassis_body, offset_pose
            )
        )
        self.gps = sens.CastToChGPSSensor(
            sens.Sensor.CreateFromJSON(
                str(sensor_path / "gps.json"), chassis_body, offset_pose
            )
        )

        self._sensors.append(self.cam)
        self._sensors.append(self.acc)
        self._sensors.append(self.gyro)
        self._sensors.append(self.mag)
        self._sensors.append(self.gps)

    def Initialize(self, sensor_manager: sens.ChSensorManager):
        self.veh.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
        self.veh.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
        self.veh.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
        self.veh.SetWheelVisualizationType(veh.VisualizationType_MESH)
        self.veh.SetTireVisualizationType(veh.VisualizationType_MESH)

        for sensor in self._sensors:
            sensor_manager.AddSensor(sensor)
        sensor_manager.Update()

    def GetSystem(self) -> chrono.ChSystem:
        return self.veh.GetSystem()
