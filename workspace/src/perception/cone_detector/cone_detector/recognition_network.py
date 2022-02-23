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
import torch
import torchvision
import torch.nn as nn
from torch import nn, Tensor
import torch.optim as optim
import torchvision.transforms as transforms

from torchvision.models.detection import *
from torchvision.models.detection.transform import GeneralizedRCNNTransform
from torchvision.models.detection.image_list import ImageList
from typing import List, Tuple, Dict, Optional

import time
import os
import glob
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from loader import *

class RecognitionNetwork():
    def __init__(self):
        # network parameters
        num_classes = 3  # background, red cones, green cones
        min_size = 720
        max_size = 1280
        box_score_thresh = 0.5
        box_detections_per_img = 100
        trainable_layers = 0
        pretrained = False
        pretrained_backbone = True
        self.device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")

        #fasterrcnn_mobilenet_v3_large_fpn
        #fasterrcnn_mobilenet_v3_large_320_fpn
        self.model = fasterrcnn_mobilenet_v3_large_320_fpn(
            pretrained=pretrained, pretrained_backbone=pretrained_backbone, 
            trainable_backbone_layers=trainable_layers, num_classes=num_classes, 
            min_size=min_size, max_size=max_size, box_score_thresh=box_score_thresh,
            box_detections_per_img=box_detections_per_img)

        self.model.transform = GeneralizedRCNNTransform(min_size,max_size,(0.485, 0.456, 0.406), (0.229, 0.224, 0.225),fixed_size=(max_size,min_size))

        self.model.to(self.device)

    def eval(self):
        # self.model.transform = DummyTransform(720,1280,(0.485, 0.456, 0.406), (0.229, 0.224, 0.225))
        self.model.eval().to(self.device)

    def predict(self, imgs):
        return self.model(imgs)

    def train_single_epoch(self, train_loader, opt, accumulation_step):
        opt.zero_grad()

        epoch_loss_total = 0

        for i, data in enumerate(train_loader):
            imgs, boxes, labels = data

            # prep the batch
            img_list = []
            target_list = []
            for b in range(imgs.size()[0]):
                img_tensor = imgs[b, :, :, :].to(self.device)
                target_dict = {}
                ids = labels[b, :] > 0
                target_dict["boxes"] = boxes[b, ids, :].to(self.device)
                target_dict["labels"] = labels[b, ids].to(self.device)
                img_list.append(img_tensor)
                target_list.append(target_dict)

            loss_dict = self.model(img_list, target_list)
            # print("loss_dict=",loss_dict)
            losses = sum(loss for loss in loss_dict.values())
            epoch_loss_total += losses.item()

            losses.backward()
            if((i + 1) % accumulation_step == 0):
                opt.step()
                opt.zero_grad()
        
        return epoch_loss_total

    def eval_dataset(self, val_loader):
        val_loss_total = 0

        for i, data in enumerate(val_loader):
            imgs, boxes, labels = data
            # prep the batch
            img_list = []
            target_list = []
            for b in range(imgs.size()[0]):
                img_tensor = imgs[b, :, :, :].to(self.device)
                target_dict = {}
                ids = labels[b, :] > 0
                # print("nonzero classes=",ids)
                target_dict["boxes"] = boxes[b, ids, :].to(self.device)
                target_dict["labels"] = labels[b, ids].to(self.device)
                img_list.append(img_tensor)
                target_list.append(target_dict)


            # fig, ax = plt.subplots()
            # ax.imshow(img_list[0].cpu().numpy().transpose((1, 2, 0)))
            # for b in boxes[0,labels[0,:]>0,:]:
            #     rect = patches.Rectangle(
            #         (b[0], b[1]), b[2]-b[0], b[3]-b[1], linewidth=1, edgecolor='b', facecolor='none')
            #     ax.add_patch(rect)
            # plt.show()

            vloss_dict = self.model(img_list, target_list)
            vlosses = sum(loss for loss in vloss_dict.values())
            val_loss_total += vlosses.item()
        return val_loss_total

    def evaluate_iou(self, data_loader, samples):
        self.eval()

        iou_list = []
        for i, data in enumerate(data_loader):
            imgs, boxes, labels = data
            self.h = imgs[0].size()[1]
            self.w = imgs[0].size()[2]

            img_list = []
            target_list = []
            for b in range(imgs.size()[0]):
                img_tensor = imgs[b, :, :, :].to(self.device)
                target_dict = {}
                ids = labels[b, :] > 0
                target_dict["boxes"] = boxes[b, ids, :].to(self.device)
                target_dict["labels"] = labels[b, ids].to(self.device)
                img_list.append(img_tensor)
                target_list.append(target_dict)

            prediction = self.predict(img_list)

            #DEBUG INFO
            # fig, ax = plt.subplots()
            # gt_boxes = target_list[0]["boxes"].cpu().detach().numpy()
            # gt_labels = target_list[0]["labels"].cpu().detach().numpy()
            # pred_boxes = prediction[0]["boxes"].cpu().detach().numpy()
            # pred_labels = prediction[0]["labels"].cpu().detach().numpy()

            # ax.imshow(img_list[0].cpu().detach().numpy().transpose((1,2,0)))
            # for b in range(gt_boxes.shape[0]):    
            #     color = 'b'
            #     rect = patches.Rectangle((gt_boxes[b, 0], gt_boxes[b, 1]), gt_boxes[b, 2]-gt_boxes[b, 0], gt_boxes[b, 3]-gt_boxes[b,1], linewidth=2, edgecolor=color, facecolor='none')
            #     ax.add_patch(rect)

            # for b in range(pred_boxes.shape[0]):    
            #     color = 'r' if int(pred_labels[b])==1 else 'g'
            #     rect = patches.Rectangle((pred_boxes[b, 0], pred_boxes[b, 1]), pred_boxes[b, 2]-pred_boxes[b, 0], pred_boxes[b, 3]-pred_boxes[b,1], linewidth=2, edgecolor=color, facecolor='none')
            #     ax.add_patch(rect)
            # plt.show()

            #perform box matching for each class
            gt_boxes = target_list[0]["boxes"].cpu().detach().numpy()
            gt_labels = target_list[0]["labels"].cpu().detach().numpy()
            pred_boxes = prediction[0]["boxes"].cpu().detach().numpy()
            pred_labels = prediction[0]["labels"].cpu().detach().numpy()

            # do matching on all classes together to include mislabeling in error metrics 
            gt_boxes = gt_boxes[gt_labels>0]
            gt_labels = gt_labels[gt_labels>0]
            pred_boxes = pred_boxes[pred_labels>0]
            pred_labels = pred_labels[pred_labels>0]

            iou_matrix = np.zeros((len(pred_labels),len(gt_labels)))

            #calculate iou for predicted box vs each gt box
            for p in range(len(pred_labels)):
                for g in range(len(gt_labels)):
                    gx0,gy0,gx1,gy1 = gt_boxes[g,:].astype(np.int32)
                    px0,py0,px1,py1 = pred_boxes[p,:].astype(np.int32)

                    #do mask comparison
                    mask = np.zeros((self.w,self.h)).astype(np.int32)
                    mask[gx0:gx1,gy0:gy1] += 1
                    mask[px0:px1,py0:py1] += 1

                    intersection = np.count_nonzero(mask[mask==2])
                    union = np.count_nonzero(mask[mask>0])

                    iou_matrix[p,g] = intersection / float(union)

            #greedily find the max IOU to match boxes
            pairs = []

            gt_ids = list(range(len(gt_labels)))
            pred_ids = list(range(len(pred_labels)))
            
            if(len(pred_labels) > 0 and len(gt_labels) > 0):
                for p in range(len(pred_labels)):
                    if(np.max(iou_matrix) < 1e-9):
                        break

                    id_max = np.unravel_index(iou_matrix.argmax(), iou_matrix.shape)
                    pairs.append(id_max)
                    pred_ids.remove(id_max[0])
                    gt_ids.remove(id_max[1])
                    #zero out the row and column selected
                    iou_matrix[id_max[0],:] = -1
                    iou_matrix[:,id_max[1]] = -1

            #handle any remaining predictions or gt boxes with no pairs
            for p in pred_ids:
                pairs.append((p,-1))
            for g in gt_ids:
                pairs.append((-1,g))      

            for p,pair in enumerate(pairs):
                iou = 0
                p_id = pair[0]
                g_id = pair[1]
                if(p_id >= 0 and g_id >= 0):
                    mask = np.zeros((self.w,self.h)).astype(np.int32)
                    mask[gx0:gx1,gy0:gy1] += 1
                    mask[px0:px1,py0:py1] += 1

                    intersection = np.count_nonzero(mask[mask==2])
                    union = np.count_nonzero(mask[mask>0])
                    iou = intersection / union
                iou_list.append(iou)

            if(i >= samples):
                break
                # print("IOU:",iou_list)
                # return np.mean(iou_list)

        # print("IOU:",iou_list)
        return np.mean(iou_list)

    def train(self, train_loader, val_loader, epochs=1, lr=.001, use_scheduler=False, accumulation_step=4, scheduler_step=1, output_path="output", save_interval=10):
        self.model.train()
        if(not os.path.exists(output_path)):
            os.mkdir(output_path)

        params = [p for p in self.model.parameters() if p.requires_grad]
        opt = optim.Adam(params, lr=lr)
        scheduler = optim.lr_scheduler.ExponentialLR(opt, gamma=0.9)

        validation_samples = 2000
        output_samples = 100

        if(val_loader==None):
            val_loader = train_loader

        for e in range(epochs):
            self.model.train()
            train_loss_total = self.train_single_epoch(train_loader, opt, accumulation_step)
            if(use_scheduler and (e+1) % scheduler_step == 0):
                scheduler.step()

            val_loss_total = self.eval_dataset(val_loader)

            train_iou = self.evaluate_iou(train_loader, len(val_loader))
            val_iou = self.evaluate_iou(val_loader, len(val_loader))


            print("Epoch [{}/{}], Lr [{:.6f}], Train Loss [{:.4f}], Val loss [{:.4f}, Train IOU [{:.4f}], Val IOU [{:.4f}]".format(e+1, epochs, scheduler.get_last_lr()[0],
                                                                                     train_loss_total / len(train_loader),
                                                                                     val_loss_total / len(val_loader),
                                                                                     train_iou, val_iou))
            if((e+1) % save_interval == 0):
                self.save(output_path)
            # # get an example prediction and show it after each epoch
            # if((e+1) % display_interval == 0):
            #     self.save(output_path)

            #     loader = train_loader if val_loader == None else val_loader
            #     loader_name = "train" if val_loader == None else "val"
            #     for i, data in enumerate(loader):
            #         imgs, boxes, labels = data
            #         img_list = []
            #         target_list = []
            #         for b in range(imgs.size()[0]):
            #             img_tensor = imgs[b, :, :, :].to(self.device)
            #             target_dict = {}
            #             target_dict["boxes"] = boxes[b, :, :].to(self.device)
            #             target_dict["labels"] = labels[b, :].to(self.device)
            #             img_list.append(img_tensor)
            #             target_list.append(target_dict)

            #         # get bounding boxes
            #         self.model.eval()
            #         pred_boxes = self.model(
            #             img_list)[0]['boxes'].detach().cpu()
            #         pred_scores = self.model(
            #             img_list)[0]['scores'].detach().cpu()
            #         pred_labels = self.model(
            #             img_list)[0]['labels'].detach().cpu()
            #         actual_box_labels = target_list[0]["labels"].detach(
            #         ).cpu().numpy()
            #         actual_boxes = target_list[0]["boxes"].detach(
            #         ).cpu().numpy()
            #         num_actual_boxes = np.count_nonzero(actual_box_labels)
            #         # print(
            #         #     "Predicted Boxes={}/{}".format(pred_boxes.size()[0], num_actual_boxes))
            #         # if(pred_boxes.size()[0] > 0):
            #         fig, ax = plt.subplots()
            #         ax.imshow(
            #             img_list[0].detach().cpu().numpy().transpose((1, 2, 0)))

            #         threshold = 0.0
            #         for b in range(pred_boxes.size()[0]):
            #             if(pred_scores[b] > threshold):
            #                 color = 'r' if pred_labels[b] == 1 else 'g'
            #                 rect = patches.Rectangle(
            #                     (pred_boxes[b, 0], pred_boxes[b, 1]), pred_boxes[b, 2]-pred_boxes[b, 0], pred_boxes[b, 3]-pred_boxes[b, 1], linewidth=1, edgecolor=color, facecolor='none')
            #                 ax.add_patch(rect)
            #                 plt.text(pred_boxes[b, 0], pred_boxes[b, 3], "{:.2f}".format(
            #                     pred_scores[b].item()), fontsize=4)
            #             # print("Max score=",torch.max(pred_scores))
            #         for b in actual_boxes[actual_box_labels>0,:]:
            #             rect = patches.Rectangle(
            #                 (b[0], b[1]), b[2]-b[0], b[3]-b[1], linewidth=1, edgecolor='b', facecolor='none')
            #             ax.add_patch(rect)
            #         plt.savefig(os.path.join(output_path,"Epoch{}_{}_sample{}.png".format(e+1, loader_name, i)), dpi=300)
            #         plt.close('all')
            #         self.model.train()
            #         if(i >= output_samples):
            #             break

    def load(self, path):
        self.model.load_state_dict(torch.load(path,map_location=self.device))
        print("Loaded pretrained model at {}".format(path))

    def save(self,output_dir="output",model_name="model"):
        if(not os.path.exists(output_dir)):
            os.mkdir(output_dir)

        torch.save(self.model.state_dict(), os.path.join(output_dir,model_name))

    def export(self,output_path="output",name="model.onnx",w=1280,h=720):
        print("Starting export to onnx")
        # if(not os.path.exists(output_path)):
        #     os.mkdir(output_path)


        # # print(self.model)
        # # print(type(self.model))

        
        # self.model.cpu()
        # self.model.train()

        # # del self.model.transform

        # #dry run for size
        # x = torch.randn(1,3,h,w).cpu()
        # self.eval()
        # self.model.cpu()



        # tmp_model = self.model


        # # print(tmp_model.transform)

        # tr = torch.nn.Sequential(
        #     transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225)),
        # )

        # tmp_model.transform = DummyTransform(720,1280,(0.485, 0.456, 0.406), (0.229, 0.224, 0.225))
        # p = tmp_model(x)
        # print(tmp_model)


        # # Export the model
        # torch.onnx.export(tmp_model,               # model being run
        #           x,                         # model input (or a tuple for multiple inputs)
        #           os.path.join(output_path,name),   # where to save the model (can be a file or file-like object)
        #           export_params=True,        # store the trained parameter weights inside the model file
        #           opset_version=11,          # the ONNX version to export the model to
        #           do_constant_folding=False,  # whether to execute constant folding for optimization
        #           input_names = ['input'],   # the model's input names
        #           output_names = ['output'])  # the model's output names)
        # print("Model exported to onnx")