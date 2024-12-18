#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@FileName       : teach_by_demonstration.py
@Time           : 2024-12-02 00:30:18
@Author         : Yan
@Email          : yding25@binghamton.edu
@Description    : Demonstration-based robot teaching script.
@Usage          : python move_eef_to_pose.py/teach_by_demonstration.py 192.168.2.100 192.168.2.108 20
"""

import time
import argparse
from Robotics_API import Bestman_Real_Flexiv
from TODO.Utils import *
import flexivrdk
import rospy

# Maximum allowable contact wrench in Cartesian space [fx, fy, fz, mx, my, mz] [N][Nm]
MAX_CONTACT_WRENCH = [50.0, 50.0, 50.0, 15.0, 15.0, 15.0]

def main():
    """
    Main function for demonstration-based robot teaching.
    """
    # Parse command-line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("robot_ip", help="IP address of the robot server")
    parser.add_argument("local_ip", help="IP address of this PC")
    parser.add_argument("frequency", help="Command frequency (1-200 Hz)", type=int)
    args = parser.parse_args()

    # Define aliases for flexivrdk modules
    log = flexivrdk.Log()
    mode = flexivrdk.Mode

    # Print tutorial description
    log.info("Tutorial description:")
    print_description()

    try:
        # Initialize ROS node
        rospy.init_node("robot_states_display", anonymous=True)

        # Create a robot interface
        bestman = Bestman_Real_Flexiv(args.robot_ip, args.local_ip, args.frequency)
        
        # Initialize the robot
        if not bestman.initialize_robot():
            return  # Exit if initialization fails

        # Teaching by demonstration variables
        recorded_poses = []
        robot_states = flexivrdk.RobotStates()

        # User input guide
        log.info("Accepted key inputs:")
        print("[n] - Start a new teaching process")
        print("[r] - Record the current robot pose")
        print("[e] - Execute the recorded poses")
        print("[q] - Quit")

        while True:
            try:
                user_input = input("Enter command ('n', 'r', 'e', 'q'): ").strip()

                if user_input == "n":
                    recorded_poses.clear()
                    bestman.robot.setMode(mode.NRT_PLAN_EXECUTION)
                    bestman.robot.executePlan("PLAN-FreeDriveAuto")
                    log.info("Started a new teaching process. Activate free-drive mode.")
                elif user_input == "r":
                    if not bestman.robot.isBusy():
                        log.warn("Please start a new teaching process first.")
                        continue
                    bestman.robot.getRobotStates(robot_states)
                    recorded_poses.append(robot_states.tcpPose)
                    log.info(f"Recorded pose: {robot_states.tcpPose}")
                    log.info(f"Total poses recorded: {len(recorded_poses)}")
                elif user_input == "e":
                    if not recorded_poses:
                        log.warn("No poses have been recorded yet.")
                        continue
                    bestman.robot.setMode(mode.NRT_PRIMITIVE_EXECUTION)
                    for idx, pose in enumerate(recorded_poses):
                        log.info(f"Executing pose {idx + 1}/{len(recorded_poses)}")
                        position = pose[:3]
                        quaternion = pose[3:]
                        euler_angles = quat2eulerZYX(quaternion, degree=True)
                        command = (
                            f"MoveCompliance(target={list2str(position)}"
                            f"{list2str(euler_angles)}WORLD WORLD_ORIGIN, "
                            f"maxVel=0.3, enableMaxContactWrench=1, "
                            f"maxContactWrench={list2str(MAX_CONTACT_WRENCH)})"
                        )
                        bestman.robot.executePrimitive(command)
                        start_time = time.time()
                        timeout = 30
                        while parse_primitive_state(bestman.robot.getPrimitiveStates(), "reachedTarget") != "1":
                            if time.time() - start_time > timeout:
                                log.error("Timeout while waiting for the robot to reach the target.")
                                break
                            time.sleep(1)
                    log.info("All poses executed. Use 'n', 'r', or 'e' for further actions.")
                    bestman.robot.setMode(mode.NRT_PLAN_EXECUTION)
                    bestman.robot.executePlan("PLAN-FreeDriveAuto")
                elif user_input == "q":
                    log.info("Exiting the program.")
                    break
                else:
                    log.warn("Invalid input. Please use 'n', 'r', 'e', or 'q'.")
            except KeyboardInterrupt:
                log.info("Program interrupted by user. Exiting...")
                break
            except Exception as e:
                log.error(f"Error: {e}")

    except Exception as e:
        log.error(f"Error: {e}")

if __name__ == "__main__":
    main()
