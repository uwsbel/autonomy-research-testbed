import rclpy
from rclpy.node import Node
from robot_localization.srv import SetDatum
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class SetDatumClient(Node):

    def __init__(self):
        super().__init__('set_datum_client')
        
        self.declare_parameter('namespace', 'artcar_1')
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value
        
        self.client = self.create_client(SetDatum, f'/{self.namespace}/datum')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.orientation = None
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.create_subscription(Imu, f'/{self.namespace}/imu/data', self.imu_callback, qos_profile)
        
        self.timer = self.create_timer(1.0, self.timer_callback)

    def imu_callback(self, msg):
        self.orientation = msg.orientation
        

    def timer_callback(self):
        if self.orientation:
            self.send_request()

    def send_request(self):
        request = SetDatum.Request()
        request.geo_pose.position.latitude = 43.07203
        request.geo_pose.position.longitude = -89.41161
        request.geo_pose.position.altitude = 260.00
        
        # Use the orientation from the IMU data
        request.geo_pose.orientation.w = 1.
        request.geo_pose.orientation.x = 0.
        request.geo_pose.orientation.y = 0.
        request.geo_pose.orientation.z = 0.
        # request.geo_pose.orientation = self.orientation

        future = self.client.call_async(request)
        future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Datum set successfully')
            self.timer.cancel()  # Stop the timer after successfully setting the datum
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = SetDatumClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
