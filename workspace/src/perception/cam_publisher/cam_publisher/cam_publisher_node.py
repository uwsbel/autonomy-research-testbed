import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from rosgraph_msgs.msg import Clock


class CamPublisher(Node):

    def __init__(self):
        super().__init__('cam_publisher')

        # create subscribers
        self.cam_sub = self.create_subscription(Image,'/sensing/front_facing_camera/raw',self.image_callback, 10)
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, 10)

        # create publishers
        self.pub = self.create_publisher(Image, '/cam0/image_raw', 10)

        # latest image recieved
        self.cam_msg = None

    def image_callback(self, msg):
        # store msg
        self.cam_msg = msg

    def clock_callback(self, msg):
        # set timestamp
        if self.cam_msg != None:
            self.cam_msg.header.stamp = msg.clock

            # publish
            self.pub.publish(self.cam_msg)
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