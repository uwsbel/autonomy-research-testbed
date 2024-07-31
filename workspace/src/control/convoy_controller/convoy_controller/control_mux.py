import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from chrono_ros_interfaces.msg import DriverInputs as VehicleInput

class CombinedInputNode(Node):
    def __init__(self):
        super().__init__('combined_input_node')
        
        self.lateral_input = 0.0
        self.long_input = 0.0
        self.braking = 0.0
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub_lateral_input = self.create_subscription(
            VehicleInput,
            '/lateral_input',
            self.lateral_input_callback,
            qos_profile
        )

        self.sub_long_input = self.create_subscription(
            VehicleInput,
            '/long_input',
            self.long_input_callback,
            qos_profile
        )

        self.pub_combined_input = self.create_publisher(
            VehicleInput,
            '/input/driver_inputs',
            10
        )

        self.timer = self.create_timer(0.1, self.publish_combined_input)

    def lateral_input_callback(self, msg):
        self.lateral_input = msg.steering

    def long_input_callback(self, msg):
        self.long_input = msg.throttle
        self.braking = msg.braking

    def publish_combined_input(self):
        combined_msg = VehicleInput()
        combined_msg.steering = self.lateral_input
        combined_msg.braking = self.braking
        combined_msg.throttle = self.long_input
        self.pub_combined_input.publish(combined_msg)
        #self.get_logger().info(f'Published combined input: Steering {self.lateral_input}, Throttle {self.long_input}.')

def main(args=None):
    rclpy.init(args=args)
    node = CombinedInputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
