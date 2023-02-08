import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from rosgraph_msgs.msg import Clock
import cv2
from cv_bridge import CvBridge, CvBridgeError


class CamPublisher(Node):

    def __init__(self):
        super().__init__('cam_publisher')

        # # get parameters
        self.declare_parameter('sim_topics', False)
        self.sim_topics = self.get_parameter('sim_topics').value
        self.declare_parameter('flip_image', False)
        self.flip_image = self.get_parameter('flip_image').value

        # create subscribers
        self.cam_sub = self.create_subscription(Image,'/sensing/front_facing_camera/raw',self.image_callback, 10)
        if self.sim_topics:
            self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, 10)

        # create publishers
        self.pub = self.create_publisher(Image, '/cam0/image_raw', 10)

        # latest image recieved
        if self.sim_topics:
            self.clock_msg = None

        self.bridge = CvBridge()

    def image_callback(self, msg):
        # from sim camera (add timestamp)
        if self.sim_topics and self.clock_msg != None:
            msg.header.stamp = self.clock_msg.clock

            # publish
            self.pub.publish(msg)
            # self.get_logger().info('Publishing: "%s"' % msg)
            return

        # from real camera (flip needed)
        if not self.sim_topics and self.flip_image:
            # flip image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image = cv2.flip(cv_image,0)
            new_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")

            # publish
            self.pub.publish(new_msg)
            # self.get_logger().info('Publishing: "%s"' % msg)
            return

        # from real camera (no flip needed)
        self.pub.publish(msg)


    def clock_callback(self, msg):
        # store msg
        self.clock_msg = msg


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