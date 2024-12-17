# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : move_arm_to_home.py
# @Time           : 2024-12-01 15:21:45
# @Author         : Yan
# @Email          : yding25@binghamton.edu
# @Description    : Move arm to default home position
# @Usage          : python move_arm_to_home.py 192.168.2.100 192.168.2.108 20
"""

import argparse
import time
from Robotics_API import Bestman_Real_Flexiv
import flexivrdk


def main():
    # Parse Arguments
    argparser = argparse.ArgumentParser(
        description="Move the robot arm to follow a trajectory."
    )
    # Required arguments
    argparser.add_argument("robot_ip", help="IP address of the robot server")
    argparser.add_argument("local_ip", help="IP address of this PC")
    argparser.add_argument("frequency", type=int, help="Command frequency, 1 to 200 [Hz]")
    args = argparser.parse_args()

    # Validate the frequency argument
    frequency = args.frequency
    assert 1 <= frequency <= 200, "Invalid <frequency> input"

    # Initialize logging
    log = flexivrdk.Log()

    try:
        # Instantiate the robot interface
        bestman = Bestman_Real_Flexiv(args.robot_ip, args.local_ip, args.frequency)

        # Initialize robot
        if not bestman.initialize_robot():
            return  # Exit if initialization fails

        # Go back to home pose
        bestman.go_home()
        time.sleep(10)

    except Exception as e:
        # Log any exceptions that occur
        log.error(str(e))


if __name__ == "__main__":
    main()
