#! /usr/bin/env python3
import sys
import argparse
from typing import Dict
import rospy

# Handle command line arguments
parser = argparse.ArgumentParser(
    description='Start network communications in ROS.')
parser.add_argument('-ip',
                    type=str,
                    help='unique static IP for ROS node')
parser.add_argument('-env',
                    type=str,
                    choices=['pi', 'sim'],
                    help='deployment environment [ pi | sim ]')


def get_run_args() -> Dict:
    NODE_NAMESPACE = rospy.get_namespace()
    NODE_NAME = rospy.get_name()
    try:
        # Fetch arguments from command line
        # Usefull when calling: rosrun coms net
        args, unknown = parser.parse_known_args()
        NODE_IP = args.ip
        NODE_ENVIRONMENT = args.env
        if len(unknown) == 0 and args.env is None or args.ip is None:
            parser.print_usage()
            sys.exit(1)
    except AttributeError:
        # Fetch arguments from launch file
        NODE_IP = rospy.get_param("/ip", "")
        NODE_ENVIRONMENT = rospy.get_param("/environment", "")
    # Validate all runtime arguments
    if NODE_NAMESPACE == "" or NODE_NAME == "" or NODE_ENVIRONMENT == "" or NODE_IP == "" or (NODE_ENVIRONMENT != "sim" and NODE_ENVIRONMENT != "pi"):
        print("Invalid command line or launch arguments.", file=sys.stderr)
        parser.print_usage()
        sys.exit(1)
    return {
        'NODE_NAMESPACE': NODE_NAMESPACE,
        'NODE_NAME': NODE_NAME,
        'NODE_IP': NODE_IP,
        'NODE_ENVIRONMENT': NODE_ENVIRONMENT
    }


def print_run_args(args: Dict) -> None:
    print("""===========================\nComs Package | RUNTIME ARGS
___________________________
NODE_NAMESPACE: {0}
NODE_NAME: {1}
NODE_IP: {2}
NODE_ENVIRONMENT: {3}\n===========================\n""".format(
        args["NODE_NAMESPACE"], args["NODE_NAME"], args["NODE_IP"], args["NODE_ENVIRONMENT"]))


def main() -> None:
    # Initialize ROS node
    rospy.init_node("coms")
    # Handle runtime arguments
    args: Dict = get_run_args()
    print_run_args(args)
    # Set fixed update-rate in Hz
    rospy.Rate(float(rospy.get_param('~rate', '2.0')))


if __name__ == "__main__":
    main()
    rospy.spin()
