'''
Run this script using:
python move_arm_with_spacemouse.py 
'''



import argparse
import time
import sys
import os
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(parent_dir, 'RoboticsToolBox'))
import pyRobotiqGripper
from Bestman_flexiv import *
from RoboticsToolBox.spacemouse import SpaceMouse, input2action, move_with_spacemouse
from RoboticsToolBox.utils import *



def main():

    argparser = argparse.ArgumentParser()
    # argparser.add_argument("--interface-cfg", type=str, default="config/charmander.yml")
    # argparser.add_argument("--controller-type", type=str, default="OSC_POSE")

    argparser.add_argument("--vendor-id", type=int, default=9583)
    argparser.add_argument("--product-id", type=int, default=50741)
    argparser.add_argument("--robot_ip", type=str, help="IP address of the robot server", default="192.168.2.100")
    argparser.add_argument("--local_ip", type=str, help="IP address of this PC", default="192.168.2.108")
    argparser.add_argument("--frequency", type=int, help="Command frequency, 1 to 200 [Hz]", default=20)
    # Optional arguments
    argparser.add_argument("--hold", action="store_true", help="Robot holds current joint positions, otherwise do a sine-sweep")
    args = argparser.parse_args()

    
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
        bestman.connect_gripper()
        time.sleep(1)
        bestman.open_gripper()
        time.sleep(1)
        move_with_spacemouse(action_num=2000, bestman=bestman, product_id=50741)


    except Exception as e:
        # Log any exceptions that occur
        log.error(str(e))

   
    #     # logger.debug(f"Time duration: {((end_time - start_time) / (10**9))}")



    

if __name__ == "__main__":
    main()
