#
# BSD 3-Clause License
#
# Copyright (c) 2022 University of Wisconsin - Madison
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.#

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from art_msgs.msg import VehicleState
from art_perception_msgs.msg import ObjectArray, Object
from ament_index_python.packages import get_package_share_directory

import tensorrt as trt
import time
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import torchvision.ops as ops
# import torchvision.transforms as transforms
import torch

from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile

import sys
import os
import json

ament_tools_root = os.path.join(os.path.dirname(__file__), '.')
sys.path.insert(0, os.path.abspath(ament_tools_root))

class YOLODetectionNode(Node):
    def __init__(self):
        super().__init__('object_recognition_node')

        # update frequency of this node
        self.freq = 50.0

        self.go = False

        # data that will be used by this class
        self.state = ""
        self.prediction = {}
        self.prediction['boxes'] = torch.tensor([])
        self.prediction['labels'] = torch.tensor([])
        self.prediction['scores'] = torch.tensor([])

        self.threshold = 0.001
        self.device = "cuda" if torch.cuda.is_available() else "cpu"

        if(not torch.cuda.is_available()):
            self.get_logger().info('cuda support in torch is not available. Either reconfigure with cuda, or fall back to a difference perception node.')
            exit(1)

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
        # self.sub_state = self.create_subscription(VehicleState, '~/input/vehicle_state', self.state_callback, qos_profile)
        self.pub_objects = self.create_publisher(ObjectArray, '~/output/objects', 10)
        self.timer = self.create_timer(1/self.freq, self.pub_callback)


        
        #initialize the TensorRT Engine
        logger = trt.Logger(trt.Logger.INFO)
        self.engine = None

        onnx_file = os.path.join(package_share_directory,self.model_file)
        engine_file = onnx_file + ".engine"
        if(os.path.exists(engine_file)):
            print("Engine file found.")
            runtime = trt.Runtime(logger)
            with open(engine_file, 'rb') as f:
                self.engine = runtime.deserialize_cuda_engine(f.read())
            
            if(self.engine == None):
                raise RuntimeError("Engine is not initialized.")

        else:
            self.get_logger().info('No existing engine file found, so building from Onnx. This may take a few minutes. Next time will be much faster.')

            if(not os.path.exists(onnx_file)):
                raise RuntimeError("Onnx file not found"+os.path.join(onnx_file))

            explicit_batch = 1 << (int)(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
            builder = trt.Builder(logger)
            network = builder.create_network(explicit_batch)
            parser = trt.OnnxParser(network, logger)
            config = builder.create_builder_config()
            config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1 << 30)

            with open(onnx_file, 'rb') as f:
                if not parser.parse(f.read()):
                    error_str = ""
                    for error in range(parser.num_errors):
                        error_str+=str(parser.get_error(error))
                    raise RuntimeError(error_str)
            self.engine = builder.build_engine(network,config) 
            
            if(self.engine == None):
                raise RuntimeError("Engine is not initialized from file: "+os.path.join(onnx_file))
            
            self.get_logger().info('Saving serialized engine for faster future loads.')
            with open(engine_file, 'wb') as f:
                f.write(self.engine.serialize())

        self.trt_context = self.engine.create_execution_context()

        self.output_shape = None
        for binding in self.engine:
            if self.engine.binding_is_input(binding):
                input_shape = self.engine.get_binding_shape(binding)
                input_size = trt.volume(input_shape) * self.engine.max_batch_size * np.dtype(np.float16).itemsize  # in bytes
            else:
                self.output_shape = self.engine.get_binding_shape(binding)

        self.confidence_thresh = .5
        self.iou_thresh = .45
        self.max_detections = 100

        self.device_output = torch.zeros(tuple(self.output_shape), device="cuda:0",dtype=torch.float16)

        self.get_logger().info('Model initialized | visualizing = %s' % (str(self.vis)))

        if(self.vis):
            matplotlib.use("TKAgg")
            self.fig, self.ax = plt.subplots()
            plt.title("Object Recognition")
            self.im_show = None
            self.patches = []
            self.counter = 0

    # function to process data this class subscribes to
    def nms(self,pred):

        #multiply class predictions by scores
        pred[:, 5:] *= pred[:, 4:5]

        #chance anchor point to corner rather than center
        boxes =  pred[:,0:4]
        boxes[:, 0] -= boxes[:, 2]/2
        boxes[:, 1] -= boxes[:, 3]/2
        boxes[:, 2] += boxes[:, 0]
        boxes[:, 3] += boxes[:, 1]

        #get the confidence and class id
        scores,class_id = pred[:, 5:].max(1, keepdims=False)

        boxes = boxes[scores>self.confidence_thresh,:]
        class_id = class_id[scores>self.confidence_thresh]
        scores = scores[scores>self.confidence_thresh]

        #have torchvision to nms
        res = ops.batched_nms(boxes=boxes, scores=scores, idxs=class_id, iou_threshold=self.iou_thresh)

        if(res.shape[0]>self.max_detections):
            res = res[:self.max_detections]

        return boxes[res,:],scores[res],class_id[res]


    def state_callback(self, msg):
        # self.get_logger().info("Received '%s'" % msg)
        self.state = msg

    def image_callback(self, msg):
        self.go = True
        # self.get_logger().info("Received image msg")
        self.image = msg

        t0 = time.time()
        
        h = self.image.height
        w = self.image.width

        img = np.asarray(self.image.data).reshape(
            h, w, -1).astype(np.float32) / 255.0
        if self.image.encoding == "bgr8":
            img = np.flip(img[:,:,0:3],axis=2)
        img = img[:704,:,0:3]
        device_input = torch.from_numpy(img).cuda().half().permute(2,0,1).unsqueeze(0)

        # img = torch.HalfTensor(self.image.data).reshape(h, w, -1) / 255.0
        # if self.image.encoding == "bgr8":
        #     img = img.flip([2])
        # img = img.cuda()
        # img = img[720-704:,:,0:3]
        # device_input = img.permute(2,0,1).unsqueeze(0) 
                
        device_input = device_input.contiguous()

        t1 = time.time()

        self.trt_context.execute_v2(bindings=[device_input.data_ptr(), self.device_output.data_ptr()])

        boxes,scores,class_id = self.nms(self.device_output[0,:,:])

        self.boxes = boxes.cpu().numpy()
        self.scores = scores.cpu().numpy()
        self.labels = class_id.cpu().numpy()

        t2 = time.time()

        if self.vis:
            if self.im_show == None:
                self.im_show = self.ax.imshow(img.astype(np.float32))
            else:
                self.im_show.set_data(img.astype(np.float32))

        t = self.get_clock().now()
        # t_msg = self.get_clock().now()
        t_msg = rclpy.time.Time.from_msg(self.image.header.stamp)
        collection_to_perception = (t.nanoseconds - t_msg.nanoseconds) / 1e9
        self.get_logger().info('Prep= %s, Inference= %s, Col2Perc= %s, ID= %s' % ("{:.4f}".format(t1-t0),"{:.4f}".format(t2-t1),"{:.4f}".format(collection_to_perception),self.image.header.frame_id))


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

        t0 = time.time()

        #clear old rectangles
        if(self.vis):
            [p.remove() for p in self.patches]
            self.patches.clear()
            [t.remove() for t in self.ax.texts]

        # self.get_logger().info('Detected %s cones' % len(self.boxes)) 
        
        for b,box in enumerate(self.boxes):
            position = self.calculate_position_from_box(box.astype(np.float64))
            obj = Object()
            obj.pose.position.x = position[0]
            obj.pose.position.y = position[1]
            obj.pose.position.z = position[2]
            obj.classification.classification = int(self.labels[b])+1
            msg.objects.append(obj)

            if(self.vis):
                color = 'r' if self.labels[b] == 0 else 'g'
                x = box[0] #- box[2]/2
                y = box[1]# - box[3]/2
                w = box[2] - box[0]
                h = box[3] - box[1]
                rect = patches.Rectangle((x,y), w,h, linewidth=1, edgecolor=color, facecolor='none')
                self.ax.add_patch(rect)
                self.patches.append(rect)

        if(self.vis):
            plt.draw()
            plt.pause(0.0001)
            self.counter += 1

        t1 = time.time()    
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_objects.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    recognition = YOLODetectionNode()
    rclpy.spin(recognition)
    recognition.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
