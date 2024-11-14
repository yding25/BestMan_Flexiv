'''
Run this script using:

python test_glass.py 192.168.2.100 192.168.2.108 20
'''

import sys
import os
# Get the correct parent directory once
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
print(os.listdir(os.path.join(parent_dir, 'Visualization')))
# Add paths without redundancies
sys.path.append(os.path.join(parent_dir, 'RoboticsToolBox'))
sys.path.append(os.path.join(parent_dir, 'Visualization'))
# Print sys.path to confirm
print('=' * 10 + "System Path" + '=' * 10)
for i, path in enumerate(sys.path):
    print(f"{i + 1}. {path}")
print('=' * 30)
from RoboticsToolBox.Bestman_flexiv import Bestman_Real_Flexiv
from RoboticsToolBox.utils import pose_to_euler, load_poses_from_xml
from camera import Camera
import numpy as np
import argparse
import time
import pygame


def main():
    # Parse Arguments
    argparser = argparse.ArgumentParser(description="Move the robot arm to follow a trajectory.")
    # Required arguments
    argparser.add_argument("robot_ip", help="IP address of the robot server")
    argparser.add_argument("local_ip", help="IP address of this PC")
    argparser.add_argument("frequency", type=int, help="Command frequency, 1 to 200 [Hz]")
    # Optional arguments
    argparser.add_argument("--hold", action="store_true", help="Robot holds current joint positions, otherwise do a sine-sweep")
    args = argparser.parse_args()

    # Validate the frequency argument
    frequency = args.frequency
    assert 1 <= frequency <= 200, "Invalid <frequency> input"

    # Instantiate the robot interface
    bestman = Bestman_Real_Flexiv(args.robot_ip, args.local_ip, args.frequency)

    try:
        # Clear fault on the robot server if any
        bestman.clear_fault()

        # Get and log current joint values and bounds
        joint_angles = bestman.get_current_joint_angles()
        bestman.log.info(f"Current joint angles: {joint_angles}")
        joint_bounds = bestman.get_joint_bounds()
        bestman.log.info(f"Current joint bounds: {joint_bounds}")

        # pose for being home 1
        pose1 = [0.5628906488418579, -0.08013617247343063, 0.4745604693889618, 0.0014537398237735033, -0.039826150983572006, 0.9992029070854187, 0.0023102618288248777]
        pose1 = pose_to_euler(pose1)
        bestman.move_end_effector_to_goal_pose(pose1)
        time.sleep(5)
        
        # Marker detection and transformation
        print('start checking RGBD')
        camera = Camera(device_id='239722070506')
        # camera.show_rgb()
        camera.display('rgbd')

    except Exception as e:
        # Log any exceptions that occur
        bestman.log.error(str(e))

if __name__ == "__main__":
    main()