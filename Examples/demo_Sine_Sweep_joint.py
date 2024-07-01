'''
Run non-real-time joint position control to hold or sine-sweep all robot joints.
'''
import sys
import os
import math
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(parent_dir, 'RoboticsToolBox'))
from Bestman_sim_flexiv import *
from utility import *


def main():
    # Parse arguments
    argparser = argparse.ArgumentParser()
    # Required arguments
    argparser.add_argument("robot_ip", help="IP address of the robot server")
    argparser.add_argument("local_ip", help="IP address of this PC")
    argparser.add_argument("frequency", help="command frequency, 1 to 200 [Hz]", type=int)
    # Optional arguments
    argparser.add_argument(
        "--hold",
        action="store_true",
        help="robot holds current joint positions, otherwise do a sine-sweep",
    )
    args = argparser.parse_args()

    # Check if arguments are valid
    frequency = args.frequency
    assert 1 <= frequency <= 200, "Invalid <frequency> input"

    # Instantiate Bestman_Real_Flexiv
    bestman = Bestman_Real_Flexiv(args.robot_ip, args.local_ip, frequency)

    try:
        # Initialize and clear faults
        bestman.Fault_clear()

        # Move robot to home pose
        bestman.go_home()
        time.sleep(5)  # Wait for the primitive to finish

        '''
        Non-real-time Joint Position Control
        '''
        period = 1.0 / frequency
        loop_time = 0
        print("Sending command to robot at", frequency, "Hz, or", period, "seconds interval")

        # Use current robot joint positions as initial positions
        init_pos = bestman.get_current_joint_values()
        print("Initial positions set to: ", init_pos)

        # Robot degrees of freedom
        DOF = bestman.get_DOF()

        # Initialize target vectors
        target_pos = init_pos.copy()
        target_vel = [0.0] * DOF
        target_acc = [0.0] * DOF

        # Joint motion constraints
        MAX_VEL = [2.0] * DOF
        MAX_ACC = [3.0] * DOF

        # Joint sine-sweep amplitude [rad]
        SWING_AMP = 0.1

        # TCP sine-sweep frequency [Hz]
        SWING_FREQ = 0.3

        # Send command periodically at user-specified frequency
        while True:
            # Use sleep to control loop period
            time.sleep(period)

            # Monitor fault on robot server
            if bestman.robot.isFault():
                raise Exception("Fault occurred on robot server, exiting ...")

            # Sine-sweep all joints
            if not args.hold:
                for i in range(DOF):
                    target_pos[i] = init_pos[i] + SWING_AMP * math.sin(
                        2 * math.pi * SWING_FREQ * loop_time
                    )
            # Otherwise all joints will hold at initial positions

            # Send command
            bestman.move_arm_to_joint_values(target_pos, target_vel, target_acc, MAX_VEL, MAX_ACC)

            # Increment loop time
            loop_time += period

    except Exception as e:
        # Print exception error message
        bestman.log.error(str(e))


if __name__ == "__main__":
    main()
