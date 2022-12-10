import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image


class CamPublisher(Node):

    def __init__(self):
        super().__init__('cam_publisher')

        # create subscribers
        self.subscriber = self.create_subscription(Image,'/sensing/front_facing_camera/raw',self.image_callback,10)

        # create publishers
        self.publisher = self.create_publisher(Image, '/cam0/image_raw', 10)

    def image_callback(self, msg):
        # add header
        msg.header.stamp = self.get_clock().now().to_msg()

        # publish
        self.publisher.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)

    cam_publisher = CamPublisher()

    rclpy.spin(cam_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cam_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()