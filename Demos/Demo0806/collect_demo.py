#!/usr/bin/env python

"""
This tutorial shows a demo implementation for teach by demonstration: free-drive the robot and
record a series of Cartesian poses, which are then reproduced by the robot.

Run this script using:
python collect_demo.py 192.168.2.100 192.168.2.108 200

200: maximum number of poses
"""

import time
import argparse
import sys
import os
import xml.etree.ElementTree as ET
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(parent_dir, 'RoboticsToolBox'))
from TODO.Utils import quat2eulerZYX, list2str, parse_pt_states
from Bestman_flexiv import *

# Maximum contact wrench [fx, fy, fz, mx, my, mz] [N][Nm]
MAX_CONTACT_WRENCH = [50.0, 50.0, 50.0, 15.0, 15.0, 15.0]

def print_description():
    """
    Print tutorial description.

    """
    print(
        "This tutorial shows a demo implementation for teach by demonstration: free-drive the "
        "robot and record a series of Cartesian poses, which are then reproduced by the robot."
    )
    print()

def save_poses_to_xml(poses, filename="recorded_traj/saved_poses4.xml"):
    """
    Save the recorded poses to an XML file.

    Args:
        poses (list): List of recorded poses.
        filename (str): Name of the XML file to save the poses.
    """
    root = ET.Element("Poses")

    for i, pose in enumerate(poses):
        pose_element = ET.SubElement(root, "Pose", id=str(i+1))
        ET.SubElement(pose_element, "Position", x=str(pose[0]), y=str(pose[1]), z=str(pose[2]))
        ET.SubElement(pose_element, "Orientation", qx=str(pose[3]), qy=str(pose[4]), qz=str(pose[5]), qw=str(pose[6]))

    tree = ET.ElementTree(root)
    tree.write(filename, encoding="utf-8", xml_declaration=True)
    print(f"Saved poses to {filename}")

def main():
    # Program Setup
    # ==============================================================================================
    # Parse arguments
    argparser = argparse.ArgumentParser()
    argparser.add_argument("robot_ip", help="IP address of the robot server")
    argparser.add_argument("local_ip", help="IP address of this PC")
    argparser.add_argument("max_num", help="Maximum number of recorded poses")
    args = argparser.parse_args()
    
    # Define alias
    log = flexivrdk.Log()
    mode = flexivrdk.Mode

    # Print description
    log.info("Tutorial description:")
    print_description()

    try:
        # RDK Initialization
        # ==========================================================================================
        # Instantiate robot interface
        robot = flexivrdk.Robot(args.robot_ip, args.local_ip)

        # Clear fault on robot server if any
        if robot.isFault():
            log.warn("Fault occurred on robot server, trying to clear ...")
            # Try to clear the fault
            robot.clearFault()
            time.sleep(2)
            # Check again
            if robot.isFault():
                log.error("Fault cannot be cleared, exiting ...")
                return
            log.info("Fault on robot server is cleared")

        # Enable the robot, make sure the E-stop is released before enabling
        log.info("Enabling robot ...")
        robot.enable()

        # Wait for the robot to become operational
        while not robot.isOperational():
            time.sleep(1)

        log.info("Robot is now operational")

        # Teach By Demonstration
        # ==========================================================================================
        # Recorded robot poses
        saved_poses = []

        # Robot states data
        robot_states = flexivrdk.RobotStates()

        # Acceptable user inputs
        log.info("Accepted key inputs:")
        print("[n] - start new teaching process")
        print("[r] - record current robot pose")
        print("[e] - finish recording and start execution")

        # User input polling
        input_buffer = ""
        while True:
            input_buffer = str(input())
            # Start new teaching process
            if input_buffer == "n":
                # Clear storage
                saved_poses.clear()

                # Put robot to plan execution mode
                robot.setMode(mode.NRT_PLAN_EXECUTION)

                # Robot run free drive
                robot.executePlan("PLAN-FreeDriveAuto")

                log.info("New teaching process started")
                log.warn(
                    "Hold down the enabling button on the motion bar to activate free drive"
                )

                # Record 500 poses
                while len(saved_poses) < int(args.max_num):
                    robot.getRobotStates(robot_states)
                    saved_poses.append(robot_states.tcpPose)
                    log.info("New pose saved: " + str(robot_states.tcpPose))
                    log.info("Number of saved poses: " + str(len(saved_poses)))
                    time.sleep(0.05)  # Delay to ensure system is not overwhelmed

                # Save poses to XML
                save_poses_to_xml(saved_poses)

            # Save current robot pose
            elif input_buffer == "r":
                if not robot.isBusy():
                    log.warn("Please start a new teaching process first")
                    continue

                robot.getRobotStates(robot_states)
                saved_poses.append(robot_states.tcpPose)
                log.info("New pose saved: " + str(robot_states.tcpPose))
                log.info("Number of saved poses: " + str(len(saved_poses)))

            # Reproduce recorded poses
            elif input_buffer == "e":
                if len(saved_poses) == 0:
                    log.warn("No pose is saved yet")
                    continue

                # Put robot to primitive execution mode
                robot.setMode(mode.NRT_PRIMITIVE_EXECUTION)

                for i in range(len(saved_poses)):
                    log.info(
                        "Executing pose " + str(i + 1) + "/" + str(len(saved_poses))
                    )

                    target_pos = [
                        saved_poses[i][0],
                        saved_poses[i][1],
                        saved_poses[i][2],
                    ]
                    # Convert quaternion to Euler ZYX required by MoveCompliance primitive
                    target_quat = [
                        saved_poses[i][3],
                        saved_poses[i][4],
                        saved_poses[i][5],
                        saved_poses[i][6],
                    ]

                    target_euler_deg = quat2eulerZYX(target_quat, degree=True)
                    robot.executePrimitive(
                        "MoveCompliance(target="
                        + list2str(target_pos)
                        + list2str(target_euler_deg)
                        + "WORLD WORLD_ORIGIN, maxVel=0.3, enableMaxContactWrench=1, maxContactWrench="
                        + list2str(MAX_CONTACT_WRENCH)
                        + ")"
                    )

                    # Wait for robot to reach target location by checking for "reachedTarget = 1"
                    # in the list of current primitive states
                    while (
                        parse_pt_states(robot.getPrimitiveStates(), "reachedTarget")
                        != "1"
                    ):
                        time.sleep(1)

                log.info(
                    "All saved poses are executed, enter 'n' to start a new "
                    "teaching process, 'r' to record more poses, 'e' to repeat "
                    "execution"
                )

                # Put robot back to free drive
                robot.setMode(mode.NRT_PLAN_EXECUTION)
                robot.executePlan("PLAN-FreeDriveAuto")
            else:
                log.warn("Invalid input")

    except Exception as e:
        # Print exception error message
        log.error(str(e))

if __name__ == "__main__":
    main()

