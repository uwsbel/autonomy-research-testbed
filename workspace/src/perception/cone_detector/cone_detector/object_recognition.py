
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from miniav_msgs.msg import VehicleState
from miniav_perception_msgs.msg import ObjectArray, Object
from ament_index_python.packages import get_package_share_directory
import torch
import torchvision
import time
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile

import sys
import os
import json

ament_tools_root = os.path.join(os.path.dirname(__file__), '.')
sys.path.insert(0, os.path.abspath(ament_tools_root))

from recognition_network import RecognitionNetwork

class ObjectRecognitionNode(Node):
    def __init__(self):
        super().__init__('object_recognition_node')

        # update frequency of this node
        self.freq = 10.0

        self.go = False

        # data that will be used by this class
        self.state = ""
        self.prediction = {}
        self.prediction['boxes'] = torch.tensor([])
        self.prediction['labels'] = torch.tensor([])
        self.prediction['scores'] = torch.tensor([])

        self.threshold = 0.001
        self.device = "cuda" if torch.cuda.is_available() else "cpu"

        # READ IN SHARE DIRECTORY LOCATION
        package_share_directory = get_package_share_directory('cone_detector')

        # READ IN PARAMETERS
        self.declare_parameter('model', "")
        self.model_file = self.get_parameter('model').get_parameter_value().string_value
        self.declare_parameter('vis', False)
        self.vis = self.get_parameter('vis').get_parameter_value().bool_value
        self.declare_parameter('camera_calibration_file', "")
        self.camera_calibration_file = self.get_parameter('camera_calibration_file').get_parameter_value().string_value

        self.camera_params = json.load(open(os.path.join(package_share_directory,self.camera_calibration_file)))
        self.get_logger().info("Cam params '%s'" % str(self.camera_params))

        # publishers and subscribers
        qos_profile = QoSProfile(depth=1)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        self.sub_image = self.create_subscription(Image, '~/input/image', self.image_callback, qos_profile)
        self.sub_state = self.create_subscription(VehicleState, '~/input/vehicle_state', self.state_callback, qos_profile)
        self.pub_objects = self.create_publisher(ObjectArray, '~/output/objects', 10)
        self.timer = self.create_timer(1/self.freq, self.pub_callback)

        # object recognition
        self.model = RecognitionNetwork()
        self.model.load(os.path.join(package_share_directory,self.model_file))
        self.model.eval()
        self.get_logger().info('Model initialized | visualizing = %s | device = %s' % (str(self.vis),str(self.device)))


        # nn optimizations
        # torch._C._jit_set_bailout_depth(1)
        # self.model.model = torch.jit.optimize_for_inference(torch.jit.script(self.model.model.eval().half())).eval().cuda()
        torch.backends.cudnn.benchmark = True
        torch.backends.cudnn.enabled = True

        if(torch.cuda.is_available()):
            self.model.model.half()

        #run a first test for optimization
        dummy_input = torch.rand((3,720,1280),dtype=torch.float32,device=self.device)
        if(torch.cuda.is_available()):
            dummy_input = torch.rand((3,720,1280),dtype=torch.float16,device=self.device)
        self.model.predict([dummy_input])

        if(self.vis):
            matplotlib.use("TKAgg")
            self.fig, self.ax = plt.subplots()
            plt.title("Object Recognition")
            self.im_show = None
            self.patches = []
            self.counter = 0

    # function to process data this class subscribes to

    def state_callback(self, msg):
        # self.get_logger().info("Received '%s'" % msg)
        self.state = msg

    def image_callback(self, msg):
        self.go = True
        # self.get_logger().info("Received '%s'" % msg)
        self.image = msg

        t0 = time.time()

        h = self.image.height
        w = self.image.width
        x = np.asarray(self.image.data).reshape(
            h, w, -1).astype(np.float32) / 255.0

        x = np.flip(x,axis=0).copy()

        if(self.image.encoding == "bgr8"):
            x = np.flip(x[:,:,0:3],axis=2).copy()

        torch_img = None 
        if(torch.cuda.is_available()):
            torch.from_numpy(x.transpose(2, 0, 1)[0:3, :, :]).half().to(self.device)
        else:
            torch.from_numpy(x.transpose(2, 0, 1)[0:3, :, :]).to(self.device)
        t1 = time.time()

        if(self.vis):
            if(self.im_show == None):
                self.im_show = self.ax.imshow(x)
            else:
                self.im_show.set_data(x)


        self.prediction = self.model.predict([torch_img])[0]

        t2 = time.time()
        t = self.get_clock().now()
        # t_msg = self.get_clock().now()
        t_msg = rclpy.time.Time.from_msg(self.image.header.stamp)
        collection_to_perception = (t.nanoseconds - t_msg.nanoseconds) / 1e9
        self.get_logger().info('Inference= %s, Col2Perc= %s, ID= %s' % ("{:.4f}".format(t2-t0),"{:.4f}".format(collection_to_perception),self.image.header.frame_id))


    def estimate_cone_distance(self, rectangle):
        return 1.0

    def direction_to_pixel(self, px):

        img_w = self.camera_params["width"]
        img_h = self.camera_params["height"]
        img_fov = self.camera_params["FOV"]

        pt_x = -((px[0]+.5) / img_w * 2 - 1) # -1 to 1
        pt_y = -((px[1]+.5) / img_h * 2 - 1) # -1 to 1 (flipped so -1 is bottom of image)
        pt_y *= img_h / img_w

        h_factor = img_fov / np.pi * 2.0

        direction = np.array([1,pt_x*h_factor,pt_y*h_factor])
        direction = direction / np.linalg.norm(direction)


        return direction

    def calculate_position_from_box(self, rectangle):

        top_px = np.array([.5*(rectangle[0] + rectangle[2]),rectangle[1]])
        bottom_px = np.array([.5*(rectangle[0] + rectangle[2]),rectangle[3]])

        r1 = self.direction_to_pixel(top_px)
        r2 = self.direction_to_pixel(bottom_px)

        h = .078

        l2 = h / ( r1[2]*r2[0] / r1[0] - r2[2] )        
        p2 = r2*l2

        mount_pos = np.asarray(self.camera_params["position"])
        mount_rot = np.asarray(self.camera_params["orientation"]).reshape((3,3))

        # self.get_logger().info('Dir: Top=(%s), Bot=(%s)' % (str(r1), str(r2)))

        p2 = np.matmul(mount_rot,p2) + mount_pos

        return p2

    # callback to run a loop and publish data this class generates
    def pub_callback(self):
        if(not self.go):
            return
        msg = ObjectArray()

        boxes = self.prediction['boxes'].detach().cpu().numpy()
        labels = self.prediction['labels'].detach().cpu().numpy()
        scores = self.prediction['scores'].detach().cpu().numpy()
        
        t0 = time.time()

        #clear old rectangles
        if(self.vis):
            [p.remove() for p in self.patches]
            self.patches.clear()
            self.ax.texts.clear()

        for b in range(boxes.shape[0]):    
            if(scores[b] > self.threshold and labels[b]>0):
                obj = Object()

                position = self.calculate_position_from_box(boxes[b, :].astype(np.float64))
                obj.pose.position.x = position[0]
                obj.pose.position.y = position[1]
                obj.pose.position.z = position[2]
                obj.classification.classification = int(labels[b])
                msg.objects.append(obj)
                
                if(self.vis):
                    color = 'r' if int(labels[b])==1 else 'g'
                    rect = patches.Rectangle((boxes[b, 0], boxes[b, 1]), boxes[b, 2]-boxes[b, 0], boxes[b, 3]-boxes[b,1], linewidth=1, edgecolor=color, facecolor='none')
                    self.ax.add_patch(rect)
                    self.patches.append(rect)

                    self.ax.text(boxes[b, 0], boxes[b, 1], "{:.2f}".format(scores[b]), fontsize=8)
                    self.ax.text(boxes[b, 0], boxes[b, 3], "{:.2f},{:.2f},{:.2f}".format(
                        position[0],position[1],position[2]), fontsize=8)

        if(self.vis):
            plt.draw()
            plt.pause(0.0001)
            self.counter += 1

        t1 = time.time()    
        # self.get_logger().info('Displaying Time = %s' % str(t1-t0))

        self.pub_objects.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    recognition = ObjectRecognitionNode()
    rclpy.spin(recognition)
    recognition.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
