# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : open_gripper.py
# @Time           : 2024-11-30 20:52:08
# @Author         : Yan
# @Email          : yding25@binghamton.edu
# @Description    : Open robotiq gripper
# @Usage          : python /home/$(whoami)/BestMan_Flexiv/Examples/open_gripper.py 192.168.2.100 192.168.2.108 20
"""

import argparse
import time
from RoboticsToolBox import Bestman_Real_Flexiv
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
    # Optional arguments
    argparser.add_argument(
        "--hold",
        action="store_true",
        help="robot holds current joint positions, otherwise do a sine-sweep",
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
        bestman.connect_gripper()
        time.sleep(1)
        bestman.open_gripper()

    except Exception as e:
        # Log any exceptions that occur
        log.error(str(e))


if __name__ == "__main__":
    main()
