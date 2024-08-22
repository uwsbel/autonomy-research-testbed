import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool  # Import the SetBool service
import numpy as np

class VehicleTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('vehicle_trajectory_publisher')
        self.odom_path = []
        self.path_length_limit = 50
        self.path_pub_distance_threshold = 0.20  # meters
        self.start_publishing_path = False  # Initially, do not publish the path
        self.pub_odom_path = self.create_publisher(Path, '/vehicle_traj', 10)
        self.sub_odometry = self.create_subscription(
            Odometry, '/odometry/filtered', self.odometry_callback, 10)
        # self.start_service = self.create_service(
            # SetBool, 'start_publishing_path', self.start_publishing_callback)

        self.go = False
        # Blocks the node thread until path publish
        self.path = self.create_subscription(
            Path,
            '/path',
            self.path_callback,
            10
        )

    def path_callback(self, msg):
        self.go = True
        return

    def odometry_callback(self, msg):
        # if self.start_publishing_path:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        self.update_odom_path(x, y, orientation)

    def update_odom_path(self, x, y, orientation):
        if not self.go:
            return

        if len(self.odom_path) == 0 or self.distance(self.odom_path[-1][0:2], (x, y)) >= self.path_pub_distance_threshold:
            self.odom_path.append((x, y, orientation))
            if len(self.odom_path) > self.path_length_limit:
                self.odom_path.pop(0)
            self.publish_odom_path()

    def publish_odom_path(self):
        if not self.go:
            return

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for (x, y, orientation) in self.odom_path:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0  # Assuming flat ground
            pose.pose.orientation = orientation
            path_msg.poses.append(pose)

        self.pub_odom_path.publish(path_msg)

    def distance(self, point1, point2):
        return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    # def start_publishing_callback(self, request, response):
    #     self.start_publishing_path = request.data
    #     response.success = True
    #     response.message = f"Publishing path set to: {self.start_publishing_path}"
    #     return response

def main(args=None):
    rclpy.init(args=args)
    node = VehicleTrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
