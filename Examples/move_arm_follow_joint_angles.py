'''
Run this script using:

python move_arm_follow_joint_angles.py 192.168.2.100 192.168.2.108 20
'''

import sys
import os
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(parent_dir, 'RoboticsToolBox'))
import pyRobotiqGripper
from Bestman_sim_flexiv import *

def main():
    # Parse Arguments
    argparser = argparse.ArgumentParser(description="Move the robot arm to follow a trajectory.")
    # Required arguments
    argparser.add_argument("robot_ip", help="IP address of the robot server")
    argparser.add_argument("local_ip", help="IP address of this PC")
    argparser.add_argument("frequency", type=int, help="Command frequency, 1 to 200 [Hz]")
    # Optional arguments
    argparser.add_argument("--hold", action="store_true", help="Robot holds current joint positions, otherwise do a sine-sweep")
    args = argparser.parse_args()

    # Validate the frequency argument
    frequency = args.frequency
    assert 1 <= frequency <= 200, "Invalid <frequency> input"

    log = flexivrdk.Log()

    try:
        # Instantiate the robot interface
        bestman = Bestman_Real_Flexiv(args.robot_ip, args.local_ip, args.frequency)

        # Clear fault on the robot server if any
        if bestman.robot.isFault():
            log.warn("Fault occurred on the robot server, trying to clear ...")
            bestman.robot.clearFault()
            time.sleep(2)
            if bestman.robot.isFault():
                log.error("Fault cannot be cleared, exiting ...")
                return
            log.info("Fault on the robot server is cleared")

        # Enable the robot, ensuring the E-stop is released before enabling
        log.info("Enabling robot ...")
        bestman.robot.enable()

        # Wait for the robot to become operational
        seconds_waited = 0
        while not bestman.robot.isOperational():
            time.sleep(1)
            seconds_waited += 1
            if seconds_waited == 10:
                log.warn(
                    "Still waiting for robot to become operational. Please "
                    "check that the robot 1) has no fault, 2) is booted "
                    "into Auto mode"
                )

        log.info("Robot is now operational")

        # Get and log current joint values and bounds
        joint_angles = bestman.get_current_joint_angles()
        log.info(f"Current joint values: {joint_angles}")

        joint_bounds = bestman.get_joint_bounds()
        log.info(f"Current joint bounds: {joint_bounds}")

        # Define the target trajectory
        target_trajectory = [
            [0.4, -0.5, 0, 1.3, 0, 0.5, 0.2],
            [0.5, -0.55, 0.1, 1.3, 0, 0.5, 0.2],
            [0.55, -0.6, 0.2, 1.3, 0, 0.5, 0.2],
            [0.6, -0.7, 0, 1.3, 0, 0.5, 0.2],
            [0.7, -0.7, 0, 1.3, 0, 0.5, 0.2],
            [0.8, -0.7, 0, 1.3, 0, 0.5, 0.2]
        ]

        # Move the arm to follow the target trajectory
        bestman.move_arm_follow_joint_angles(target_trajectory)
        time.sleep(1)

    except Exception as e:
        # Log any exceptions that occur
        log.error(str(e))


if __name__ == "__main__":
    main()
