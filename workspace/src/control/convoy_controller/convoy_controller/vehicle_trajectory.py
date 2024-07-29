import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np

class VehicleTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('vehicle_trajectory_publisher')
        self.odom_path = []
        self.path_length_limit = 30
        self.path_pub_distance_threshold = 1.0  # meters
        self.start_publishing_path = False
        self.pub_odom_path = self.create_publisher(Path, '/vehicle_traj', 10)
        self.sub_odometry = self.create_subscription(
            Odometry, '/odometry/filtered', self.odometry_callback, 10)

    def odometry_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.update_odom_path(x, y)

    def update_odom_path(self, x, y):
        if len(self.odom_path) == 0 or self.distance(self.odom_path[-1], (x, y)) >= self.path_pub_distance_threshold:
            self.odom_path.append((x, y))
            if len(self.odom_path) > self.path_length_limit:
                self.odom_path.pop(0)
            self.publish_odom_path()

    def publish_odom_path(self):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for (x, y) in self.odom_path:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0  # Assuming flat ground
            path_msg.poses.append(pose)

        self.pub_odom_path.publish(path_msg)

    def distance(self, point1, point2):
        return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

def main(args=None):
    rclpy.init(args=args)
    node = VehicleTrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
