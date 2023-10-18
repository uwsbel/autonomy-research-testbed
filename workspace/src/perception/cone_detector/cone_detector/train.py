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
import time
from recognition_network import *
from loader import *
from metrics import *
import argparse
import sys

def generate_boxes(args):

    train_dataset = SegImgLoader(data_root=args.training_path,
                                 max_samples=args.n_train)

    train_dataset.GenerateAAVBBFromSeg()


def train(args):
    # run training as requested
    print("=== Loading Datasets ===")

    train_dataset = ObjectDetectionImgLoader(data_root=args.training_path, max_boxes=args.max_boxes,
                                             max_samples=args.n_train,apply_transforms=True)
    val_dataset = ObjectDetectionImgLoader(data_root=args.validation_path, max_boxes=args.max_boxes,
                                           max_samples=args.n_val,apply_transforms=False)

    train_loader = torch.utils.data.DataLoader(
        train_dataset, args.batch_size, shuffle=True, num_workers=args.threads, drop_last=True)
    val_loader = torch.utils.data.DataLoader(
        val_dataset, args.batch_size, shuffle=True, num_workers=args.threads, drop_last=True)

    print("=== Training ===")

    model = RecognitionNetwork()
    if(os.path.exists(args.input_model)):
        model.load(args.input_model)
    model.train(train_loader, val_loader, epochs=args.epochs,
                lr=args.learning_rate, use_scheduler=args.use_scheduler,
                accumulation_step=args.acc_step, scheduler_step=args.sched_step,
                output_path=args.output_path, save_interval=args.save_interval)

    # model.export(output_path=args.output_path,name=args.name+".onnx",w=1280,h=720)

    print("=== Training Complete ===")

def main(args):
    #save the configuration to a bash script
    if(not os.path.exists(args.output_path)):
        os.mkdir(args.output_path)

    run_file = open(os.path.join(args.output_path,"run.sh"),'w')
    run_file.write("#!/usr/bin/env bash\n")
    command = "python3 "
    for a in sys.argv:
        command += a + " "
    run_file.write(command + "\n")
    run_file.close()


    if(args.mode == "train"):
        train(args)
    elif(args.mode == "generate_boxes"):
        generate_boxes(args)


def parseargs():

    # === DEFAULTS === #
    training_path = "/home/asher/datasets/chrono_cones/train"
    validation_path = "/home/asher/datasets/chrono_cones/val"

    lr = 1e-3
    epochs = 6
    batch_size = 12
    display_interval = 1
    print_interval = 1
    loading_threads = 12
    max_boxes = 100
    num_training_samples = -1
    num_validation_samples = -1
    num_testing_samples = -1
    avail_model_types = ["fasterrcnn_mobilenet_v3_large_320_fpn"]
    model_type = avail_model_types[0]
    output_path = "output"
    input_model = "input/model"
    acc_step = 4
    sched_step = 1

    parser = argparse.ArgumentParser(description='Object Recognition Trainer.')

    # general mode
    parser.add_argument('--mode', default="train",
                        choices=["train", "generate_boxes"], help='mode of use')

    # information about the network
    parser.add_argument('--name', type=str, default='model',
                        help="network name for saving and loading")
    parser.add_argument('--model_type', default=model_type,
                        choices=avail_model_types, help='the object recognition model to use')
    parser.add_argument('--output_path', type=str,
                        default=output_path, help="output path for model and data")
    parser.add_argument('--input_model', type=str, default=input_model,
                        help="a pretrained model to use for continued training")

    # training parameters
    parser.add_argument('--epochs', '-e', type=int,
                        default=epochs, help="number of epochs for training")
    parser.add_argument('--batch_size', '-b', type=int,
                        default=batch_size, help="samples per batch")
    parser.add_argument('--learning_rate', '-lr', type=float,
                        default=lr, help="learning rate")
    parser.add_argument('--use_scheduler', action='store_true',
                        help="use a learning rate scheduler to decrease learning rate during training")
    parser.add_argument('--save_interval', type=int, default=display_interval,
                        help="number of epochs between displaying and saving progress during training")
    parser.add_argument('--threads', type=int, default=loading_threads,
                        help="worker threads for loading data")
    parser.add_argument('--max_boxes', type=int, default=max_boxes,
                        help="maximum number of predicted boxes possible in an image")
    parser.add_argument('--acc_step', type=int, default=acc_step,
                        help="accumulation steps for gradient calculations")
    parser.add_argument('--sched_step', type=int, default=sched_step,
                        help="scheduler steps for learning rate changes")

    # dataset locations
    parser.add_argument('--training_path', '-tr_data', type=str,
                        default=training_path, help="path to training data")
    parser.add_argument('--validation_path', '-val_data', type=str,
                        default=validation_path, help="path to validation data")

    # number of samples to load
    parser.add_argument('--n_train', type=int, default=num_training_samples,
                        help="number of samples to load for training")
    parser.add_argument('--n_val', type=int, default=num_validation_samples,
                        help="number of samples to load for validation")

    return parser.parse_args()


if __name__ == "__main__":
    args = parseargs()
    main(args)
