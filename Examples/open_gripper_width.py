'''
python /home/$(whoami)/BestMan_Flexiv/Examples/open_gripper.py 192.168.2.100 192.168.2.108 20
'''

import sys
import os
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(parent_dir, 'RoboticsToolBox'))
import pyRobotiqGripper
from Bestman_sim_flexiv import *

def main():
    # Parse Arguments
    # =============================================================================
    argparser = argparse.ArgumentParser()
    # Required arguments
    argparser.add_argument("robot_ip", help="IP address of the robot server (default: 192.168.2.100)")
    argparser.add_argument("local_ip", help="IP address of this PC")
    argparser.add_argument(
        "frequency", help="command frequency, 1 to 200 [Hz]", type=int)
    # Optional arguments
    argparser.add_argument(
        "--hold", action="store_true",
        help="robot holds current joint positions, otherwise do a sine-sweep")
    args = argparser.parse_args()

    # Check if arguments are valid
    frequency = args.frequency
    assert (frequency >= 1 and frequency <= 200), "Invalid <frequency> input"

    log = flexivrdk.Log()


    try:
        '''
        RDK Initialization
        '''
        # Instantiate robot interface
        bestman = Bestman_Real_Flexiv(args.robot_ip, args.local_ip, args.frequency)
        
        # Clear fault on robot server if any
        if bestman.robot.isFault():
            log.warn("Fault occurred on robot server, trying to clear ...")
            
            # Try to clear the fault
            bestman.robot.clearFault()
            time.sleep(1)
            
            # Check again
            if bestman.robot.isFault():
                log.error("Fault cannot be cleared, exiting ...")
                return
            
            log.info("Fault on robot server is cleared")
        
        # Enable the robot, make sure the E-stop is released before enabling
        log.info("Enabling robot ...")
        bestman.robot.enable()
        
        # Wait for the robot to become operational
        seconds_waited = 0
        while not bestman.robot.isOperational():
            time.sleep(1)
            seconds_waited += 1
            if seconds_waited == 10:
                log.warn(
                    "Still waiting for robot to become operational, please "
                    "check that the robot 1) has no fault, 2) is booted "
                    "into Auto mode")
        log.info("Robot is now operational")

        # wait until the action is done
        while (parse_pt_states(bestman.robot.getPrimitiveStates(), "reachedTarget") != "1"):
            time.sleep(1)

        # open gripper
        bestman.connect_gripper()
        bestman.gripper_goto(value=100)
        time.sleep(1)

    except Exception as e:
        # Print exception error message
        log.error(str(e))


if __name__ == "__main__":
    main()
