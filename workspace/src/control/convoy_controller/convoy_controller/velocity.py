import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry  # Import for Odometry messages
from art_msgs.msg import VehicleInput  # Replace with the actual import path for VehicleInput
from rclpy.qos import QoSProfile, QoSHistoryPolicy

class PID:
    def __init__(self, kp, ki, kd, setpoint=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.integral = 0.0
        self.previous_error = 0.0

    def compute(self, measured_value, dt):
        error = self.setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        self.previous_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class VelocityPIDNode(Node):
    def __init__(self):
        super().__init__('velocity_pid_node')

        # PID parameters
        self.kp = 0.6 #self.declare_parameter('kp', 1.0).get_parameter_value().double_value
        self.ki = 0.01 #self.declare_parameter('ki', 0.0).get_parameter_value().double_value
        self.kd = -0.005 #self.declare_parameter('kd', 0.0).get_parameter_value().double_value

        # PID controller for velocity
        self.pid = PID(self.kp, self.ki, self.kd)

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to /cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            qos_profile
        )
        
        # Subscribe to /odometry/filtered for current velocity
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            qos_profile
        )
        
        # Publisher for VehicleInput (Throttle and Steering)
        self.input_pub = self.create_publisher(VehicleInput, '/input/driver_inputs', 10)
        
        # Current velocity, target velocity, and steering angle
        self.current_velocity = 0.0
        self.target_velocity = 0.0
        self.steering_angle = 0.0
        
        # Timer for periodic PID control updates
        self.timer = self.create_timer(0.05, self.timer_callback)  # 10 Hz update rate
        
    def cmd_vel_callback(self, msg):
        self.target_velocity = msg.linear.x
        self.steering_angle = msg.angular.z
        self.pid.setpoint = self.target_velocity
    
    def odom_callback(self, msg):
        # Extract the linear velocity in the x-direction from the odometry message
        self.current_velocity = msg.twist.twist.linear.x
    
    def timer_callback(self):
        dt = 0.05  # Time step (s)
        
        # Compute throttle using PID controller
        throttle_output = self.pid.compute(self.current_velocity, dt)
        
        # Publish the control command
        vehicle_input_msg = VehicleInput()
        vehicle_input_msg.throttle = max(0.0, min(throttle_output, 1.0))  # Constrain between 0 and 1
        vehicle_input_msg.steering = self.steering_angle  # Use the steering angle from /cmd_vel
        
        self.input_pub.publish(vehicle_input_msg)
        
        self.get_logger().info(f"Current velocity: {self.current_velocity}, Target: {self.target_velocity}, Throttle: {vehicle_input_msg.throttle}, Steering: {self.steering_angle}")

def main(args=None):
    rclpy.init(args=args)
    node = VelocityPIDNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
