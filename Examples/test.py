# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : open_gripper.py
# @Time           : 2024-11-30 20:52:08
# @Author         : Yan
# @Email          : yding25@binghamton.edu
# @Description    : Open robotiq gripper
# @Usage          : python test.py 192.168.2.100 192.168.2.108 20
"""

import argparse
import time
from Robotics_API import Bestman_Real_Flexiv
import flexivrdk


def main():
    # Parse Arguments
    argparser = argparse.ArgumentParser()
    # Required arguments
    argparser.add_argument(
        "robot_ip", help="IP address of the robot server (default: 192.168.2.100)"
    )
    argparser.add_argument("local_ip", help="IP address of this PC")
    argparser.add_argument(
        "frequency", help="command frequency, 1 to 200 [Hz]", type=int
    )
    args = argparser.parse_args()

    # Check if arguments are valid
    frequency = args.frequency
    assert frequency >= 1 and frequency <= 200, "Invalid <frequency> input"

    # Initialize logging
    log = flexivrdk.Log()

    try:
        # Instantiate robot interface
        bestman = Bestman_Real_Flexiv(args.robot_ip, args.local_ip, args.frequency)

        # Initialize robot
        if not bestman.initialize_robot():
            return  # Exit if initialization fails

        # Open gripper
        current_pose = bestman.get_current_eef_pose()
        current_joint = bestman.get_current_joint_values()
        print(f'gt current_pose:{current_pose.position} and {current_pose.orientation}, current_joint:{current_joint}')

        # bestman.move_arm_to_joint_values(current_joint)

        predicted_joint = bestman.cartesian_to_joints(current_pose)
        bestman.move_arm_to_joint_values(predicted_joint)
        print(f'predicted joint :{predicted_joint}')

        # predicted_pose = bestman.joints_to_cartesian(current_joint)
        # bestman.move_eef_to_goal_pose(predicted_pose)
        # print(f'predicted pose :{predicted_pose}')

    except Exception as e:
        # Log any exceptions that occur
        log.error(str(e))


if __name__ == "__main__":
    main()
