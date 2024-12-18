# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : close_gripper.py
# @Time           : 2024-11-30 20:52:08
# @Author         : Yan & Zhaxi
# @Email          : yding25@binghamton.edu & zhaxizhuoma.ayang@gmail.com
# @Description    : Open robotiq gripper
# @Usage          : python close_gripper.py 192.168.2.100 192.168.2.108 20
"""

import argparse
import time
from Robotics_API import Bestman_Real_Flexiv

def main():
    argparser = argparse.ArgumentParser()
    argparser.add_argument("robot_ip", help="IP address of the robot server (default: 192.168.2.100)")
    argparser.add_argument("local_ip", help="IP address of this PC")
    argparser.add_argument("frequency", help="command frequency, 1 to 200 [Hz]", type=int)
    args = argparser.parse_args()

    frequency = args.frequency
    assert frequency >= 1 and frequency <= 200, "Invalid <frequency> input"

    try:
        # Instantiate robot interface
        bestman = Bestman_Real_Flexiv(args.robot_ip, args.local_ip, args.frequency)

        # Initialize robot
        if not bestman.initialize_robot():
            return  # Exit if initialization fails

        # Open gripper
        bestman.connect_gripper()
        time.sleep(1)
        bestman.close_gripper()

    except Exception as e:
        print(str(e))


if __name__ == "__main__":
    main()
