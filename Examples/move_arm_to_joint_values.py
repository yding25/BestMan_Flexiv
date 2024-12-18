# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : move_arm_to_joint_values.py
# @Time           : 2024-11-30 20:52:08
# @Author         : Yan
# @Email          : yding25@binghamton.edu
# @Description    : Move arm to targeted joint values
# @Usage          : python move_arm_to_joint_values.py 192.168.2.100 192.168.2.108 20
"""

import argparse
import rospy
from Robotics_API import Bestman_Real_Flexiv
import flexivrdk
import threading

def wait_and_log_motion(bestman, target_joint):
    if bestman.wait_for_motion_completion(target_joint, error_threshold=0.01, speed_threshold=0.01, timeout=10):
        rospy.loginfo("Robot motion completed successfully.")
    else:
        rospy.logwarn("Robot motion did not complete within the timeout.")

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
        
        # Move the arm to a set of joint values

        # target_joint = [0.8, -0.7, 0, 1.3, 0, 0.5, 0.2]

        import math
        target_joint = [0, -40, 0, 90, 0, 80, 0]
        # target_joint = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
        target_joint = [math.radians(target_joint[n]) for n in range(7)]

        bestman.move_arm_to_joint_values(target_joint)
        
        # Wait for motion completion (This method will block the main threa)
        if bestman.wait_for_joints(target_joint, error_threshold=0.01, speed_threshold=0.01, timeout=10):
            rospy.loginfo("Robot motion completed successfully.")
        else:
            rospy.logwarn("Robot motion did not complete within the timeout.")
        
        # # Wait for motion completion (This method will NOT block the main threa)
        # motion_thread = threading.Thread(target=wait_and_log_motion, args=(bestman, target_joint))
        # motion_thread.start()

    except Exception as e:
        # Log any exceptions that occur
        log.error(str(e))


if __name__ == "__main__":
    main()
