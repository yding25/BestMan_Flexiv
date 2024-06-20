import sys
import os

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(parent_dir, 'RoboticsToolBox'))

from Api_flexiv import *
import math


# Global constants
# ==================================================================================================
# TCP sine-sweep amplitude [m]
SWING_AMP = 0.1

# TCP sine-sweep frequency [Hz]
SWING_FREQ = 0.3

# External TCP force threshold for collision detection, value is only for demo purpose [N]
EXT_FORCE_THRESHOLD = 10.0

# External joint torque threshold for collision detection, value is only for demo purpose [Nm]
EXT_TORQUE_THRESHOLD = 5.0

def print_description():
    """
    Print tutorial description.
    """
    print(
        "This tutorial runs non-real-time Cartesian-space pure motion control to hold or "
        "sine-sweep the robot TCP. A simple collision detection is also included."
    )
    print()

def main():
    # Program Setup
    # ==============================================================================================
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
    robot_controller = Bestman_Real_Flexiv(args.robot_ip, args.local_ip, frequency)

    # Print description
    print_description()

    # Print based on arguments
    if args.hold:
        robot_controller.log.info("Robot holding current TCP pose")
    else:
        robot_controller.log.info("Robot running TCP sine-sweep")

    if args.collision:
        robot_controller.log.info("Collision detection enabled")
    else:
        robot_controller.log.info("Collision detection disabled")

    try:
        # Initialize and clear faults
        robot_controller.Fault_clear()

        # Move robot to home pose
        robot_controller.go_home()
        time.sleep(5)  # Wait for the primitive to finish

        # Zero Force-torque Sensor
        # =========================================================================================
        robot_controller.robot.executePrimitive("ZeroFTSensor()")
        robot_controller.log.warn(
            "Zeroing force/torque sensors, make sure nothing is in contact with the robot"
        )

        # Wait for primitive completion
        while robot_controller.robot.isBusy():
            time.sleep(1)
        robot_controller.log.info("Sensor zeroing complete")

        # Configure Motion Control
        # =========================================================================================
        # Switch to non-real-time mode for discrete motion control
        robot_controller.robot.setMode(robot_controller.mode.NRT_CARTESIAN_MOTION_FORCE)

        # Set initial pose to current TCP pose
        robot_controller.update_robot_states()
        init_pose = robot_controller.get_current_end_effector_pose()
        print("Initial TCP pose set to [position 3x1, rotation (quaternion) 4x1]: ", init_pose)

        # Periodic Task
        # =========================================================================================
        # Set loop period
        period = 1.0 / frequency
        loop_counter = 0
        print("Sending command to robot at", frequency, "Hz, or", period, "seconds interval")

        # Send command periodically at user-specified frequency
        while True:
            # Use sleep to control loop period
            time.sleep(period)

            # Monitor fault on robot server
            if robot_controller.robot.isFault():
                raise Exception("Fault occurred on robot server, exiting ...")

            # Read robot states
            robot_controller.update_robot_states()
            robot_states = robot_controller.robot_states

            # Initialize target pose to initial pose
            target_pose = init_pose.copy()

            # Sine-sweep TCP along Y axis
            if not args.hold:
                target_pose[1] = init_pose[1] + SWING_AMP * math.sin(
                    2 * math.pi * SWING_FREQ * loop_counter * period
                )
            # Otherwise robot TCP will hold at initial pose

            # Move end effector to target pose
            robot_controller.move_end_effector_to_goal_pose(target_pose)

            # Do the following operations in sequence for every 20 seconds
            time_elapsed = loop_counter * period
            if time_elapsed % 20.0 == 3.0:
                preferred_jnt_pos = [0.938, -1.108, -1.254, 1.464, 1.073, 0.278, -0.658]
                robot_controller.robot.setNullSpacePosture(preferred_jnt_pos)
                robot_controller.log.info("Preferred joint positions set to: ")
                print(preferred_jnt_pos)
            elif time_elapsed % 20.0 == 6.0:
                new_K = np.multiply(robot_controller.robot.info().nominalK, 0.5)
                robot_controller.robot.setCartesianStiffness(new_K)
                robot_controller.log.info("Cartesian stiffness set to: ")
                print(new_K)
            elif time_elapsed % 20.0 == 9.0:
                preferred_jnt_pos = [-0.938, -1.108, 1.254, 1.464, -1.073, 0.278, 0.658]
                robot_controller.robot.setNullSpacePosture(preferred_jnt_pos)
                robot_controller.log.info("Preferred joint positions set to: ")
                print(preferred_jnt_pos)
            elif time_elapsed % 20.0 == 12.0:
                robot_controller.robot.resetCartesianStiffness()
                robot_controller.log.info("Cartesian stiffness is reset")
            elif time_elapsed % 20.0 == 14.0:
                robot_controller.robot.resetNullSpacePosture()
                robot_controller.log.info("Preferred joint positions are reset")
            elif time_elapsed % 20.0 == 16.0:
                max_wrench = [10.0, 10.0, 10.0, 2.0, 2.0, 2.0]
                robot_controller.robot.setMaxContactWrench(max_wrench)
                robot_controller.log.info("Max contact wrench set to: ")
                print(max_wrench)
            elif time_elapsed % 20.0 == 19.0:
                robot_controller.robot.resetMaxContactWrench()
                robot_controller.log.info("Max contact wrench is reset")

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
                    robot_controller.robot.stop()
                    robot_controller.log.warn("Collision detected, stopping robot and exit program ...")
                    return

            # Increment loop counter
            loop_counter += 1

    except Exception as e:
        # Print exception error message
        robot_controller.log.error(str(e))

if __name__ == "__main__":
    main()