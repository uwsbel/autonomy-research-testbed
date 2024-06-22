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

# import torch.nn as nn
# import torch.optim as optim
import torchvision.transforms as transforms

import time
import os
import glob
import numpy as np
from PIL import Image, ImageEnhance, ImageFilter
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# import cv2


class SegImgLoader:
    def __init__(self, data_root, max_samples=-1, img_format=".png", seg_format=".png"):
        self.imgs = []
        self.seg_imgs = []
        self.data_root = data_root
        self.img_dir = os.path.join(data_root, "imgs")
        self.seg_dir = os.path.join(data_root, "seg_imgs")
        self.max_samples = max_samples

        if not os.path.exists(self.img_dir):
            print("Error, img directory not found: {}".format(self.img_dir))
            exit(1)

        if not os.path.exists(self.seg_dir):
            print("Error, segmented img directory not found: {}".format(self.seg_dir))
            exit(1)

        self.imgs = glob.glob(self.img_dir + "/*" + img_format)
        if len(self.imgs) == 0:
            print(
                "Error: no images found in {}".format(
                    os.path.join(self.data_root, "imgs/*" + img_format)
                )
            )
            exit(1)

        for f in self.imgs:
            basename = os.path.splitext(os.path.basename(f))[0]
            self.seg_imgs.append(os.path.join(self.seg_dir, basename + seg_format))

        if self.max_samples > 0 and self.max_samples < len(self.imgs):
            self.imgs = self.imgs[0 : self.max_samples]
            self.seg_imgs = self.seg_imgs[0 : self.max_samples]

        print(
            "Data loaded. Imgs={}, Seg Imgs={}".format(
                len(self.imgs), len(self.seg_imgs)
            )
        )

    def ConvertSegToBoxes(self, semantic_maps):
        # boxes = np.asarray(
        #     [0, 0, 1, 1]*self.max_boxes).reshape((self.max_boxes, 4))
        # labels = np.zeros(self.max_boxes).reshape(
        #     (self.max_boxes)).astype(np.int64)

        boxes = []
        labels = []

        box_count = 0

        for c in range(1, np.max(semantic_maps[:, :, 0]) + 1):
            for i in range(1, np.max(semantic_maps[:, :, 1]) + 1):
                indices = np.where(
                    np.logical_and(
                        semantic_maps[:, :, 1] == i, semantic_maps[:, :, 0] == c
                    )
                )
                if indices[0].shape[0] > 1:
                    y0 = np.min(indices[0])
                    y1 = np.max(indices[0])
                    x0 = np.min(indices[1])
                    x1 = np.max(indices[1])

                    if x1 > x0 and y1 > y0:
                        # change x0,y0,x1,y1 to normalized center (x,y) and width, height
                        x_center = 0.5 * (x0 + x1) / float(semantic_maps.shape[1])
                        y_center = 0.5 * (y0 + y1) / float(semantic_maps.shape[0])
                        x_size = (x1 - x0) / float(semantic_maps.shape[1])
                        y_size = (y1 - y0) / float(semantic_maps.shape[0])

                        # boxes.append(np.array([x0, y0, x1, y1]))
                        boxes.append(np.array([x_center, y_center, x_size, y_size]))
                        labels.append(c)
                        box_count += 1

        boxes = np.asarray(boxes)  # .astype(np.int32)
        labels = np.asarray(labels).astype(np.int32)
        return boxes, labels

    def GenerateAAVBBFromSeg(self, label_format=".txt"):
        label_dir = os.path.join(self.data_root, "labels")
        if not os.path.exists(label_dir):
            os.mkdir(label_dir)

        for i in range(len(self.imgs)):
            # load segmentation img
            seg_img = np.array(Image.open(self.seg_imgs[i])).view(np.uint16)[:, :, :]

            # generate boxes and labels from segmentation img
            boxes, classes = self.ConvertSegToBoxes(seg_img)

            if len(classes) > 0:
                classes -= np.ones(classes.shape).astype(np.int32)
                classes = np.reshape(classes, (len(classes), 1))
                output = np.append(classes, boxes, axis=1)

                basename = os.path.splitext(os.path.basename(self.imgs[i]))[0]
                file_name = os.path.join(label_dir, basename + label_format)
                np.savetxt(file_name, output, fmt="%.6f")

                print("Generated AABB file {}/{}".format(i + 1, len(self.imgs)))


class ObjectDetectionImgLoader(torch.utils.data.Dataset):
    def __init__(
        self,
        data_root,
        max_boxes,
        apply_transforms=False,
        max_samples=-1,
        img_format=".png",
        box_format=".txt",
    ):
        self.name = "Object Detection Image Loader"
        self.data_root = data_root
        self.max_boxes = max_boxes
        self.max_samples = max_samples

        self.img_dir = os.path.join(self.data_root, "imgs")
        self.label_dir = os.path.join(self.data_root, "labels")

        self.imgs = []
        self.labels = []

        # transform parameters
        self.use_transforms = apply_transforms
        np.random.seed(1)
        self.flip_prob = 0.5
        self.max_translation = (0.2, 0.2)
        self.brightness = (0.75, 1.33)
        self.sharpness = (0.25, 4.0)
        self.saturation = (0.75, 1.33)
        self.contrast = (0.75, 1.33)
        self.max_zoom = 2.0

        if not os.path.exists(self.data_root):
            print("Error: directory not found. Data root = {}".format(self.data_root))
            exit(1)

        if not os.path.exists(self.img_dir):
            print(
                "Error: directory not found. Image directory = {}".format(self.img_dir)
            )
            exit(1)

        if not os.path.exists(self.label_dir):
            print(
                "Error: directory not found. Label directory = {}".format(
                    self.label_dir
                )
            )
            exit(1)

        self.imgs = glob.glob(os.path.join(self.data_root, "imgs/*"))

        if len(self.imgs) == 0:
            print(
                "Error: no images found in {}".format(
                    os.path.join(self.data_root, "imgs/*")
                )
            )
            exit(1)

        for f in self.imgs:
            basename = os.path.splitext(os.path.basename(f))[0]
            self.labels.append(os.path.join(self.label_dir, basename + box_format))

        if self.max_samples > 0 and self.max_samples < len(self.imgs):
            self.imgs = self.imgs[0 : self.max_samples]
            self.labels = self.labels[0 : self.max_samples]

        print(
            "Data loaded. Imgs={}, Labels={}".format(len(self.imgs), len(self.labels))
        )

    def __len__(self):
        return len(self.imgs)

    def ApplyTransforms(self, img, boxes, classes):
        # get height and width parameters
        height = np.asarray(img).shape[0]
        width = np.asarray(img).shape[1]

        # === random horizontal flip ===
        if np.random.rand() > self.flip_prob:
            # flip image horizontally
            img = img.transpose(Image.FLIP_LEFT_RIGHT)

            # flip boxes horizontally
            boxes_x_0 = width - 1 - boxes[:, 0]
            boxes_x_1 = width - 1 - boxes[:, 2]
            boxes[:, 0] = boxes_x_1
            boxes[:, 2] = boxes_x_0

        # === random zoom and crop ===
        zoom = np.random.uniform(1.0, self.max_zoom)
        img = img.resize(
            (int(zoom * width), int(zoom * height)), resample=Image.BILINEAR
        )
        crop_pt = np.random.uniform(0.0, 0.9, (2))
        # crop_pt = [0,0]
        crop_pt = (crop_pt * (zoom * width - width, zoom * height - height)).astype(
            np.int
        )

        crop_pt[0] = np.clip(crop_pt[0], 0, img.size[0] - width)
        crop_pt[1] = np.clip(crop_pt[1], 0, img.size[1] - height)

        # img = img[crop_pt[1],crop_pt[1]+height,crop_pt[0],crop_pt[0]+width,:]
        img = img.crop(
            (crop_pt[0], crop_pt[1], crop_pt[0] + width, crop_pt[1] + height)
        )
        boxes = (boxes * zoom).astype(np.int)
        boxes = boxes - (crop_pt[0], crop_pt[1], crop_pt[0], crop_pt[1])
        boxes[:, 0] = np.clip(boxes[:, 0], 0, width - 1)  # clip x0 value
        boxes[:, 2] = np.clip(boxes[:, 2], 0, width - 1)  # clip x1 value
        boxes[:, 1] = np.clip(boxes[:, 1], 0, height - 1)  # clip y0 value
        boxes[:, 3] = np.clip(boxes[:, 3], 0, height - 1)  # clip y1 value
        for i in range(len(classes)):
            # if box moved out of image, set label to 0
            if (
                abs(boxes[i, 0] - boxes[i, 2]) < 0.5
                or abs(boxes[i, 1] - boxes[i, 3]) < 0.5
            ):
                classes[i] = 0  # reset label
                boxes[i, :] = np.array([0, 1, 0, 1])  # reset to valid box coords

        # === random translation ===
        # get translation parameters
        t_x, t_y = (
            np.random.uniform(-1, 1, size=2) * self.max_translation * (height, width)
        )
        t_x = int(t_x)
        t_y = int(t_y)
        # apply translation to img
        img = img.transform(img.size, Image.AFFINE, (1, 0, t_x, 0, 1, t_y))
        # img = img.rotate(1,translate=(t_x,t_y))
        # apply translation to boxes, cutting any that fully leave the image
        boxes = boxes - np.array([t_x, t_y, t_x, t_y])
        boxes[:, 0] = np.clip(boxes[:, 0], 0, width - 1)  # clip x0 value
        boxes[:, 2] = np.clip(boxes[:, 2], 0, width - 1)  # clip x1 value
        boxes[:, 1] = np.clip(boxes[:, 1], 0, height - 1)  # clip y0 value
        boxes[:, 3] = np.clip(boxes[:, 3], 0, height - 1)  # clip y1 value
        for i in range(len(classes)):
            # if box moved out of image, set label to 0
            if (
                abs(boxes[i, 0] - boxes[i, 2]) < 0.5
                or abs(boxes[i, 1] - boxes[i, 3]) < 0.5
            ):
                classes[i] = 0  # reset label
                boxes[i, :] = np.array([0, 1, 0, 1])  # reset to valid box coords

        # === random brightness, hue, saturation changes ===
        brighten = ImageEnhance.Brightness(img)
        img = brighten.enhance(
            np.random.uniform(self.brightness[0], self.brightness[1], size=1)
        )

        sharpen = ImageEnhance.Sharpness(img)
        img = sharpen.enhance(
            np.random.uniform(self.sharpness[0], self.sharpness[1], size=1)
        )

        saturate = ImageEnhance.Color(img)
        img = saturate.enhance(
            np.random.uniform(self.saturation[0], self.saturation[1], size=1)
        )

        contrast = ImageEnhance.Contrast(img)
        img = saturate.enhance(
            np.random.uniform(self.contrast[0], self.contrast[1], size=1)
        )

        return img, boxes, classes

    def __getitem__(self, idx):
        if torch.is_tensor(idx):
            idx = idx.tolist()

        # load files
        img = Image.open(self.imgs[idx])
        boxes = np.asarray([0.5, 0.5, 0.2, 0.2] * self.max_boxes).reshape(
            (self.max_boxes, 4)
        )
        classes = np.zeros(self.max_boxes)

        # if the boxes file doesn't exist, it means there were no boxes in that image
        if not os.path.exists(self.labels[idx]):
            # ensure correct datatypes
            img = np.asarray(img) / 255.0
            img = np.transpose(img, (2, 0, 1)).astype(np.float32)[0:3, :, :]
            boxes = np.asarray([0, 0, 1, 1] * self.max_boxes).reshape(
                (self.max_boxes, 4)
            )
            boxes = boxes.astype(np.int32)
            classes = classes.astype(np.int64)
            return img, boxes, classes

        classes_and_boxes = np.loadtxt(self.labels[idx]).reshape(-1, 5)

        classes[0 : classes_and_boxes.shape[0]] = classes_and_boxes[:, 0] + 1
        boxes[0 : classes_and_boxes.shape[0], :] = classes_and_boxes[:, 1:5]

        # convert normalized boxes to index-based boxes
        height = np.asarray(img).shape[0]
        width = np.asarray(img).shape[1]

        center_x = boxes[:, 0].copy()
        center_y = boxes[:, 1].copy()
        size_x = boxes[:, 2].copy()
        size_y = boxes[:, 3].copy()

        boxes[:, 0] = np.clip(np.round((center_x - size_x / 2) * width), 0, width - 2)
        boxes[:, 1] = np.clip(np.round((center_y - size_y / 2) * height), 0, height - 2)
        boxes[:, 2] = np.clip(
            np.round((center_x + size_x / 2) * width), boxes[:, 0] + 1, width - 1
        )
        boxes[:, 3] = np.clip(
            np.round((center_y + size_y / 2) * height), boxes[:, 1] + 1, height - 1
        )

        # ensure correct datatypes and formats
        boxes = boxes.astype(np.int32)
        classes = classes.astype(np.int64)

        # apply transforms
        if self.use_transforms:
            img, boxes, classes = self.ApplyTransforms(img, boxes, classes)

        # TEMPORARILY BLUR IMAGES
        # img = img.filter(ImageFilter.GaussianBlur(radius=1))

        # ensure correct image format and channels first
        img = np.asarray(img) / 255.0
        img = np.transpose(img, (2, 0, 1)).astype(np.float32)[0:3, :, :]

        # fig, ax = plt.subplots()
        # ax.imshow(img.transpose((1, 2, 0)))
        # for b in boxes[classes>0,:]:
        #     rect = patches.Rectangle(
        #         (b[0], b[1]), b[2]-b[0], b[3]-b[1], linewidth=1, edgecolor='b', facecolor='none')
        #     ax.add_patch(rect)
        # plt.show()

        # cv2_img = cv2.cvtColor(img.transpose((1, 2, 0)), cv2.COLOR_RGB2BGR)
        # cv2.imshow("training image", cv2_img)
        # cv2.waitKey(0) # waits until a key is pressed
        # cv2.destroyAllWindows() # destroys the window showing image

        return img, boxes, classes
