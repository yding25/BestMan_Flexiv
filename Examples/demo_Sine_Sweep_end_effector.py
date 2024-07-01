'''
Run Cartesian-space pure motion control to hold or sine-sweep the robot TCP. 
A simple collision detection is also included.
'''

import sys
import os
import math
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(parent_dir, 'RoboticsToolBox'))
from Bestman_sim_flexiv import *

'''
Global constant
'''
# TCP sine-sweep amplitude [m]
SWING_AMP = 0.1
# TCP sine-sweep frequency [Hz]
SWING_FREQ = 0.3
# External TCP force threshold for collision detection, value is only for demo purpose [N]
EXT_FORCE_THRESHOLD = 10.0
# External joint torque threshold for collision detection, value is only for demo purpose [Nm]
EXT_TORQUE_THRESHOLD = 5.0

def main():
    # Parse arguments
    argparser = argparse.ArgumentParser()
    # Required arguments
    argparser.add_argument("robot_ip", help="IP address of the robot server")
    argparser.add_argument("local_ip", help="IP address of this PC")
    argparser.add_argument("frequency", help="command frequency, 1 to 100 [Hz]", type=int)
    # Optional arguments
    argparser.add_argument(
        "--hold",
        action="store_true",
        help="robot holds current TCP pose, otherwise do a sine-sweep",
    )
    argparser.add_argument(
        "--collision",
        action="store_true",
        help="enable collision detection, robot will stop upon collision",
    )
    args = argparser.parse_args()

    # Check if arguments are valid
    frequency = args.frequency
    assert 1 <= frequency <= 100, "Invalid <frequency> input"

    # Instantiate Bestman_Real_Flexiv
    bestman = Bestman_Real_Flexiv(args.robot_ip, args.local_ip, frequency)

    # Print based on arguments
    if args.hold:
        bestman.log.info("Robot holding current TCP pose")
    else:
        bestman.log.info("Robot running TCP sine-sweep")

    if args.collision:
        bestman.log.info("Collision detection enabled")
    else:
        bestman.log.info("Collision detection disabled")

    try:
        # Initialize and clear faults
        bestman.Fault_clear()

        # Move robot to home pose
        bestman.go_home()
        time.sleep(5)  # Wait for the primitive to finish

        # Zero Force-torque Sensor
        bestman.robot.executePrimitive("ZeroFTSensor()")
        bestman.log.warn(
            "Zeroing force/torque sensors, make sure nothing is in contact with the robot"
        )
        
        # Wait for primitive completion
        while bestman.robot.isBusy():
            time.sleep(1)
        bestman.log.info("Sensor zeroing complete")

        # Switch to non-real-time mode for discrete motion control
        bestman.robot.setMode(bestman.mode.NRT_CARTESIAN_MOTION_FORCE)

        # Set initial pose to current TCP pose
        bestman.update_robot_states()
        init_pose = bestman.get_current_end_effector_pose()
        print("Initial TCP pose set to [position 3x1, rotation (quaternion) 4x1]: ", init_pose)

        # Set loop period
        period = 1.0 / frequency
        loop_counter = 0
        print("Sending command to robot at", frequency, "Hz, or", period, "seconds interval")

        # Send command periodically at user-specified frequency
        while True:
            # Use sleep to control loop period
            time.sleep(period)

            # Monitor fault on robot server
            if bestman.robot.isFault():
                raise Exception("Fault occurred on robot server, exiting ...")

            # Read robot states
            bestman.update_robot_states()
            robot_states = bestman.robot_states

            # Initialize target pose to initial pose
            target_pose = init_pose.copy()

            # Sine-sweep TCP along Y axis
            if not args.hold:
                target_pose[1] = init_pose[1] + SWING_AMP * math.sin(
                    2 * math.pi * SWING_FREQ * loop_counter * period
                )
            # Otherwise robot TCP will hold at initial pose

            # Move end effector to target pose
            bestman.move_end_effector_to_goal_pose(target_pose)

            # Do the following operations in sequence for every 20 seconds
            time_elapsed = loop_counter * period
            if time_elapsed % 20.0 == 3.0:
                preferred_jnt_pos = [0.938, -1.108, -1.254, 1.464, 1.073, 0.278, -0.658]
                bestman.robot.setNullSpacePosture(preferred_jnt_pos)
                bestman.log.info("Preferred joint positions set to: ")
                print(preferred_jnt_pos)
            elif time_elapsed % 20.0 == 6.0:
                new_K = np.multiply(bestman.robot.info().nominalK, 0.5)
                bestman.robot.setCartesianStiffness(new_K)
                bestman.log.info("Cartesian stiffness set to: ")
                print(new_K)
            elif time_elapsed % 20.0 == 9.0:
                preferred_jnt_pos = [-0.938, -1.108, 1.254, 1.464, -1.073, 0.278, 0.658]
                bestman.robot.setNullSpacePosture(preferred_jnt_pos)
                bestman.log.info("Preferred joint positions set to: ")
                print(preferred_jnt_pos)
            elif time_elapsed % 20.0 == 12.0:
                bestman.robot.resetCartesianStiffness()
                bestman.log.info("Cartesian stiffness is reset")
            elif time_elapsed % 20.0 == 14.0:
                bestman.robot.resetNullSpacePosture()
                bestman.log.info("Preferred joint positions are reset")
            elif time_elapsed % 20.0 == 16.0:
                max_wrench = [10.0, 10.0, 10.0, 2.0, 2.0, 2.0]
                bestman.robot.setMaxContactWrench(max_wrench)
                bestman.log.info("Max contact wrench set to: ")
                print(max_wrench)
            elif time_elapsed % 20.0 == 19.0:
                bestman.robot.resetMaxContactWrench()
                bestman.log.info("Max contact wrench is reset")

            # Simple collision detection: stop robot if collision is detected at end-effector
            if args.collision:
                collision_detected = False
                ext_force = np.array(robot_states.extWrenchInBase[:3])
                if np.linalg.norm(ext_force) > EXT_FORCE_THRESHOLD:
                    collision_detected = True

                for v in robot_states.tauExt:
                    if abs(v) > EXT_TORQUE_THRESHOLD:
                        collision_detected = True

                if collision_detected:
                    bestman.robot.stop()
                    bestman.log.warn("Collision detected, stopping robot and exit program ...")
                    return

            # Increment loop counter
            loop_counter += 1

    except Exception as e:
        # Print exception error message
        bestman.log.error(str(e))

if __name__ == "__main__":
    main()