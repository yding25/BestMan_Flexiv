# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : move_eef_to_pose.py
# @Time           : 2024-12-01 22:29:10
# @Author         : Yan
# @Email          : yding25@binghamton.edu
# @Description    : XXX
# @Usage          : python move_eef_to_pose.py 192.168.2.100 192.168.2.108 20
"""

import argparse
import rospy
from Robotics_API import Bestman_Real_Flexiv, Pose
import flexivrdk
import time

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
        # Initialize ROS node
        rospy.init_node("robot_states_display", anonymous=True)

        # Instantiate robot interface
        bestman = Bestman_Real_Flexiv(args.robot_ip, args.local_ip, args.frequency)
        
        # Initialize robot
        if not bestman.initialize_robot():
            return  # Exit if initialization fails
        
        # Define the target trajectory
        target_pose = Pose([0.6121770143508911, 0.04117409512400627, 0.2725994288921356], [0.4189033806324005, -0.5498623847961426, 0.6234208345413208, -0.36540088057518005])
        
        # Move the arm to follow the target trajectory
        bestman.move_eef_to_goal_pose(target_pose)
        
        # Wait for motion completion (This method will block the main threa)
        bestman.wait_for_eef(target_pose)

    except Exception as e:
        # Log any exceptions that occur
        log.error(str(e))


if __name__ == "__main__":
    main()
