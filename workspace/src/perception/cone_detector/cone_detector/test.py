import time
from recognition_network import *
from loader import *
from metrics import *
import argparse
import sys

def test(args):

    test_dataset = ObjectDetectionImgLoader(data_root=args.testing_path,max_boxes=args.max_boxes,
        max_samples=args.n,apply_transforms=False)
    test_loader = torch.utils.data.DataLoader(
        test_dataset, 1, shuffle=False, num_workers=args.threads, drop_last=True)

    model = RecognitionNetwork()
    if(os.path.exists(args.input_model)):
        model.load(args.input_model)
    model.eval()

    tester = MetricTester(model, test_loader, args.output_path,args.name)
    tester.RunTest(classes=[1,2])
    tester.SaveMetrics(display=True)

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

    test(args)

def parseargs():

    # === DEFAULTS === #
    testing_path = "/home/asher/datasets/chrono_cones/test"
    num_testing_samples = -1
    avail_model_types = ["fasterrcnn_mobilenet_v3_large_320_fpn"]
    model_type = avail_model_types[0]
    output_path = "output"
    input_model = "input/model"
    max_boxes = 100
    loading_threads = 12

    parser = argparse.ArgumentParser(description='Object Recognition Tester.')

    # information about the network
    parser.add_argument('--name', type=str, default='model',
                        help="network name for saving and loading")
    parser.add_argument('--model_type', default=model_type,
                        choices=avail_model_types, help='the object recognition model to use')
    parser.add_argument('--output_path', type=str,
                        default=output_path, help="output path for model and data")
    parser.add_argument('--input_model', type=str, default=input_model,
                        help="a pretrained model to use for continued training or testing")
    parser.add_argument('--max_boxes', type=int, default=max_boxes,
                        help="maximum number of predicted boxes possible in an image")
    parser.add_argument('--threads', type=int, default=loading_threads,
                        help="worker threads for loading data")

    #testing parameters
    #TODO

    # dataset locations
    parser.add_argument('--testing_path', '-data', type=str,
                        default=testing_path, help="path to test data")

    # number of samples to load
    parser.add_argument('--n', type=int, default=num_testing_samples,
                        help="number of samples to load for testing")

    return parser.parse_args()


if __name__ == "__main__":
    args = parseargs()
    main(args)
