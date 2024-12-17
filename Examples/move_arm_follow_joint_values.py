# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : move_arm_follow_joint_values.py
# @Time           : 2024-12-01 20:13:39
# @Author         : Yan
# @Email          : yding25@binghamton.edu
# @Description    : XXX
# @Usage          : python move_arm_follow_joint_values.py 192.168.2.100 192.168.2.108 20
"""

import argparse
import rospy
from Robotics_API import Bestman_Real_Flexiv
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
        target_trajectory = [
            [0.4, -0.5, 0, 1.3, 0, 0.5, 0.2],
            [0.5, -0.55, 0.1, 1.3, 0, 0.5, 0.2],
            [0.55, -0.6, 0.2, 1.3, 0, 0.5, 0.2],
            [0.6, -0.7, 0, 1.3, 0, 0.5, 0.4],
            [0.6, -0.7, 0, 1.3, 0, 0.5, 0.2],
            [0.7, -0.7, 0, 1.3, 0, 0.5, 0.2],
            [0.8, -0.7, 0, 1.3, 0, 0.5, 0.2]
        ]

        # Move the arm to follow the target trajectory
        for i in range(len(target_trajectory)):
            bestman.move_arm_to_joint_values(target_trajectory[i])

            # Wait for motion completion (This method will block the main threa)
            if bestman.wait_for_joints(target_trajectory[i]):
                rospy.loginfo("Robot motion completed successfully.")
            else:
                rospy.logwarn("Robot motion did not complete within the timeout.")
        

    except Exception as e:
        # Log any exceptions that occur
        log.error(str(e))


if __name__ == "__main__":
    main()
