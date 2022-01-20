

import numpy as np
import sys, os, time

#tensorrt
# import tensorrt as trt
# import pycuda.driver as cuda
# import pycuda.autoinit

#torch
import torch


from recognition_network import *

# import onnxruntime as ort


# print("Testing tensor rt")

# onnx_file_path = "output/model.onnx"
torch_file_path = "../data/model_refined"

n = 100

def test_torch(): 

    #optimizations to environment
    torch.backends.cudnn.benchmark = True
    torch.backends.cudnn.enabled = True
    print(torch.backends.cudnn.allow_tf32)

    # with torch.no_grad():

    model = RecognitionNetwork()
    model.load(torch_file_path)
    # model.model.cpu()
    # model.model.to(memory_format=torch.channels_last)

    model.eval()

    model.model = model.model.half()

    # torch._C._jit_set_bailout_depth(1)
    # # self.model.model = torch.jit.optimize_for_inference(torch.jit.script(self.model.model.eval())).eval().cuda()
    # model.model = torch.jit.optimize_for_inference(torch.jit.script(model.model))
    # torch.backends.cudnn.benchmark = True
    # torch.backends.cudnn.enabled = True

    imgs = np.random.rand(n,3,720,1280).astype(np.float16)

    # imgs = torch.rand(n,3,720,1280, device=torch.device('cuda:0')).half()

    # inputs = torch.from_numpy(imgs[0:1,:,:,:]).half().cuda()

    print(torch.from_numpy(imgs[0,:,:,:]).size())

    inputs = torch.from_numpy(imgs[0,:,:,:]).cuda()

    # inputs = [torch.rand(3,720,1280).cuda()]
    # print("is pinned:",inputs.is_pinned())
    pred = model.predict([inputs])
    
    t0 = time.time()
    for i in range(n):
        # inputs = torch.from_numpy(imgs[i:i+1,:,:,:]).half().cuda()
        inputs = [torch.from_numpy(imgs[i,:,:,:]).cuda()]
        # inputs = torch.from_numpy(imgs[0,:,:,:]).cuda()
        # pred = model.predict([inputs])/
        model.predict(inputs)
        # print("is pinned:",inputs.is_pinned())
    t1 = time.time()
    print("=== Mean torch inference time = {}".format((t1-t0)/n))


# def test_onnx():

#     ort_session = ort.InferenceSession(onnx_file_path)
#     n = 100
#     imgs = np.random.rand(n,3,720,1280).astype(np.float32)
#     name = ort_session.get_inputs()[0].name

#     #dry run to remove setup time
#     inputs = {name: imgs[0:1,:,:,:]}
#     pred = ort_session.run(None,inputs)

#     t0 = time.time()
#     for i in range(imgs.shape[0]):
#         inputs = {name: imgs[i:i+1,:,:,:]}
#         pred = ort_session.run(None,inputs)
#     t1 = time.time()

#     print("=== Mean onnx inference time = {}".format((t1-t0)/n))

# TRT_LOGGER = trt.Logger()
# onnx_file_path = "output/model.onnx"
# engine_file_path = "output/trt_engine"
# EXPLICIT_BATCH = 1 << (int)(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)

# def test_tensorrt():

#     with trt.Builder(TRT_LOGGER) as builder, builder.create_network(EXPLICIT_BATCH) as network, builder.create_builder_config() as config, trt.OnnxParser(network, TRT_LOGGER) as parser, trt.Runtime(TRT_LOGGER) as runtime:
#         config.max_workspace_size = 1 << 28 # 256MiB
#         builder.max_batch_size = 1
#         # Parse model file
#         if not os.path.exists(onnx_file_path):
#             print('ONNX file {} not found.'.format(onnx_file_path))
#             exit(0)
#         print('Loading ONNX file from path {}...'.format(onnx_file_path))
#         with open(onnx_file_path, 'rb') as model:
#             print('Beginning ONNX file parsing')
#             if not parser.parse(model.read()):
#                 print ('ERROR: Failed to parse the ONNX file.')
#                 for error in range(parser.num_errors):
#                     print (parser.get_error(error))
#                 return None
#         # The actual yolov3.onnx is generated with batch size 64. Reshape input to batch size 1
#         # network.get_input(0).shape = [1, 3, 608, 608]
#         print('Completed parsing of ONNX file')
#         print('Building an engine from file {}; this may take a while...'.format(onnx_file_path))
#         plan = builder.build_serialized_network(network, config)
#         engine = runtime.deserialize_cuda_engine(plan)
#         print("Completed creating Engine")
#         with open(engine_file_path, "wb") as f:
#             f.write(plan)
#         return engine


#     pass

test_torch()
# test_tensorrt()
# test_onnx()
