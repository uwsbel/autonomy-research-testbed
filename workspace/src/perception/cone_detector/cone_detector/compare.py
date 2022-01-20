import time
from metrics import *
import argparse
import sys

    
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

    comp = MetricComparitor(args.model_name,args.a_name,args.b_name,args.a_path,args.b_path)
    comp.RunConeSimilarityMetric()
    
    # comp.GenerateComparisonPlots(save=args.save,display=args.display,output_path=args.output_path)


def parseargs():

    # === DEFAULTS === #
    a_metrics_path = "output_a"
    b_metrics_path = "output_b"
    max_comparison_samples = -1

    parser = argparse.ArgumentParser(description='Object Recognition Trainer.')

    # information about the network
    parser.add_argument('--model_name', type=str, default='model',
                        help="network name that is performing the comparison")

    #information about the datasets being compared
    parser.add_argument('--a_name', type=str, default='Dataset A',
                        help="Name of first dataset in the comparison")
    parser.add_argument('--a_path', type=str, default=a_metrics_path,
                        help="Path to first dataset's metrics files")

    parser.add_argument('--b_name', type=str, default='Dataset B',
                        help="Name of second dataset in the comparison")
    parser.add_argument('--b_path', type=str, default=b_metrics_path,
                        help="Path to second dataset's metrics files")
    
    #miscellaneous parameters
    parser.add_argument("--display",action="store_true",help="Display the comparisons")
    parser.add_argument('--n', type=int, default=max_comparison_samples,
                        help="number of samples to load for training")
    parser.add_argument("--save",action="store_true",help="Save the comparisons")
    parser.add_argument("--output_path",type=str, default='output_comparison',
                        help="Output Directory for Comparisons")

    return parser.parse_args()


if __name__ == "__main__":
    args = parseargs()
    main(args)
