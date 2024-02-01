import rclpy
from rclpy.node import Node
from rclpy.time import Time
from chrono_ros_interfaces.msg import Body
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped


class PositionSubscriber(Node):
    def __init__(self):
        super().__init__('position_subscriber')
        self.subscription = self.create_subscription(
            PoseStamped,  
            '/chrono_ros_node/output/vehicle/state/pose',  
            self.listener_callback,
            10)
        self.subscription
        self.x_values = []
        self.y_values = []
        self.last_message_time = self.get_clock().now()

    def listener_callback(self, msg):
        self.last_message_time = self.get_clock().now()
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.x_values.append(x)
        self.y_values.append(y)

def main(args=None):
    rclpy.init(args=args)
    position_subscriber = PositionSubscriber()

    # Main loop with timeout check
    while rclpy.ok():
        rclpy.spin_once(position_subscriber, timeout_sec=0.1)  # Short spin to handle callbacks
        current_time = position_subscriber.get_clock().now()
        if (current_time - position_subscriber.last_message_time).nanoseconds > 5e9:  # 5 seconds in nanoseconds
            break

    rclpy.shutdown()

    # Plotting
    plt.scatter(position_subscriber.x_values, position_subscriber.y_values)
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Position Plot')
    
    plt.savefig('NN_Controller.png')
    plt.show()

if __name__ == '__main__':
    main()
