# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : graspany.py
# @Time           : 2024-12-01 22:29:10
# @Author         : Yan
# @Email          : yding25@binghamton.edu
# @Description    : XXX
# @Usage          : python graspany.py 192.168.2.100 192.168.2.108 20
"""

import argparse
import rospy
from Robotics_API import Bestman_Real_Flexiv, Pose
import flexivrdk
import time
import math

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
        
        # target_joint = [0, -20, 0, 110, 0, 40, 0]
        # target_joint = [math.radians(target_joint[n]) for n in range(7)]
        # target_pose = bestman.joints_to_cartesian(target_joint)
        
        # bestman.move_arm_to_joint_values(target_joint)
        # bestman.wait_for_joints(target_joint, error_threshold=0.01, speed_threshold=0.01, timeout=10)

        # cartesian_matrix = bestman.robot_chain.forward_kinematics([0]+target_joint+[0])
        # print(cartesian_matrix)

        target_pose = Pose([0.40, 0.00, 0.30], [0, 0, 1, 0]) 
        bestman.move_eef_to_goal_pose(target_pose)
        bestman.wait_for_eef(target_pose)


        # Define the target trajectory
        # target_pose = Pose([0.57406, -0.10835, 0.21324], [0.4189033806324005, -0.5498623847961426, 0.6234208345413208, -0.36540088057518005]) # 四元数
        # target_pose = Pose([0.57406, -0.10835, 0.21324], [math.radians(0), math.radians(180), math.radians(00)]) # 欧拉角（弧度）
        
        # Move the arm to follow the target trajectory
        # bestman.move_eef_to_goal_pose(target_pose)
        
        # Wait for motion completion (This method will block the main threa)
        # bestman.wait_for_eef(target_pose)

    except Exception as e:
        # Log any exceptions that occur
        log.error(str(e))


if __name__ == "__main__":
    main()
