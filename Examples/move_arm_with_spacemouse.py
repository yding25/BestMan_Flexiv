'''
Run this script using:
python move_arm_with_spacemouse.py 192.168.2.100 192.168.2.108 20
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
    argparser.add_argument("--product-id", type=int, default=50734)
    argparser.add_argument("robot_ip", help="IP address of the robot server")
    argparser.add_argument("local_ip", help="IP address of this PC")
    argparser.add_argument("frequency", type=int, help="Command frequency, 1 to 200 [Hz]")
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

        move_with_spacemouse(bestman)
        #connect gripper
    #     bestman.connect_gripper()
    #     time.sleep(1)
    #     bestman.open_gripper()
    #     time.sleep(1)
    #     last_gripper_state = -1.0

    #     #initial the pose
    #     bestman.go_home()
    #     time.sleep(3)
        
    #     # Get and log current joint values and bounds
    #     for i in range(1000):
    #         # start_time = time.time_ns()

    #         action, grasp = input2action(
    #             device=device
    #         )
    #         print(action)
    #         action[3] = -action[3]
    #         action[4] = -action[4]
    #         current_gripper_state = action[6]
    #         if current_gripper_state != last_gripper_state:
    #             if current_gripper_state == 1:
    #                 bestman.close_gripper()
    #             else:
    #                 bestman.open_gripper()
    #             last_gripper_state = current_gripper_state
    #         current_pos = bestman.get_current_end_effector_pose()
    #         target_pos = current_pos + action[0:6]
    #         print(target_pos, action[6])
    #         bestman.move_end_effector_to_goal_pose(target_pos, max_linear_vel=0.05, max_angular_vel=0.3)
    #         time.sleep(0.05)
            
    #         # end_time = time.time_ns()

    except Exception as e:
        # Log any exceptions that occur
        log.error(str(e))

   
    #     # logger.debug(f"Time duration: {((end_time - start_time) / (10**9))}")



    

if __name__ == "__main__":
    main()
