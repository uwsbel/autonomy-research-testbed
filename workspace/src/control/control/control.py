import rclpy
from rclpy.node import Node
from miniav_msgs.msg import VehicleState, VehicleInput
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from ament_index_python.packages import get_package_share_directory
import numpy as np
import os

from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # DEFAULT SETTINGS

        # control node mode
        self.mode = "PID"  # "PID", "File"
        self.file = ""
        self.recorded_inputs = np.array([])

        # update frequency of this node
        self.freq = 10.0

        self.t_start = self.get_clock().now().nanoseconds / 1e9

        # READ IN SHARE DIRECTORY LOCATION
        package_share_directory = get_package_share_directory('control')

        # ROS PARAMETERS
        self.declare_parameter('control_mode', 'PID')
        self.mode = self.get_parameter('control_mode').get_parameter_value().string_value
        self.declare_parameter('control_file', "")
        self.file = self.get_parameter('control_file').get_parameter_value().string_value

        self.declare_parameter('steering_gain', 1.0)
        self.steering_gain = self.get_parameter('steering_gain').get_parameter_value().double_value
        self.declare_parameter('throttle_gain', 1.0)
        self.throttle_gain = self.get_parameter('throttle_gain').get_parameter_value().double_value


        if(self.file == ""):
            self.mode = "PID"
        else:
            file_path = os.path.join(package_share_directory, self.file)
            self.recorded_inputs = np.loadtxt(file_path, delimiter=',')

        self.steering = 0.0
        self.throttle = 0.0
        self.braking = 0.0

        # data that will be used by this class
        self.state = ""
        self.path = Path()
        self.vehicle_cmd = VehicleInput()

        #waits for first path if using PID, otherwise runs right away
        self.go = (self.mode == "File")
        

        # publishers and subscribers
        qos_profile = QoSProfile(depth=1)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        self.sub_path = self.create_subscription(
            Path, 'miniav/path', self.path_callback, qos_profile)
        self.sub_state = self.create_subscription(
            VehicleState, 'miniav/state', self.state_callback, qos_profile)
        self.pub_vehicle_cmd = self.create_publisher(
            VehicleInput, 'miniav/vehicle_cmd', 10)
        self.timer = self.create_timer(1/self.freq, self.pub_callback)

    # function to process data this class subscribes to
    def state_callback(self, msg):
        # self.get_logger().info("Received '%s'" % msg)
        self.state = msg

    def path_callback(self, msg):
        self.go = True
        # self.get_logger().info("Received '%s'" % msg)
        self.path = msg

    # callback to run a loop and publish data this class generates
    def pub_callback(self):
        if(not self.go):
            return

        msg = VehicleInput()

        if(self.mode == "File"):
            self.calc_inputs_from_file()
        elif(self.mode == "PID" and len(self.path.poses)>0):
            pt = [self.path.poses[0].pose.position.x,self.path.poses[0].pose.position.y]
            
            ratio = pt[1] / pt[0]
            self.steering = self.steering_gain * ratio
            # self.get_logger().info('Target steering = %s' % self.steering)

        #TODO: remove after debugging
        # self.steering = 0.0
        self.throttle = self.throttle_gain*0.55 #only doing lateral conmtrol for now
        # self.braking = 0.0

        msg.steering = np.clip(self.steering, -1, 1)
        msg.throttle = np.clip(self.throttle, 0, 1)
        msg.braking = np.clip(self.braking, 0, 1)
        self.pub_vehicle_cmd.publish(msg)
        

    def calc_inputs_from_file(self):
        t = self.get_clock().now().nanoseconds / 1e9 - self.t_start

        self.throttle = np.interp(t,self.recorded_inputs[:,0],self.recorded_inputs[:,1])
        self.braking = np.interp(t,self.recorded_inputs[:,0],self.recorded_inputs[:,2])
        self.steering = np.interp(t,self.recorded_inputs[:,0],self.recorded_inputs[:,3])

        # self.get_logger().info('Inputs %s' % self.recorded_inputs[0,:])

        # self.get_logger().info('Inputs from file: (t=%s, (%s,%s,%s)),' % (t,self.throttle,self.braking,self.steering))

def main(args=None):
    rclpy.init(args=args)
    control = ControlNode()
    rclpy.spin(control)

    control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
