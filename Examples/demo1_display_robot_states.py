import sys
import os

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(parent_dir, 'RoboticsToolBox'))

from Api_flexiv import *



def print_description():
    """
    Print tutorial description.
    """
    print(
        "This tutorial does the very first thing: check connection with the robot server "
        "and print received robot states."
    )
    print()

def print_robot_states(robot_controller):
    """
    Print robot states data @ 1Hz.
    """
    while True:
        # Get the latest robot states
        robot_controller.update_robot_states()
        robot_states = robot_controller.robot_states

        # Print all robot states, round all float values to 2 decimals
        print("{")
        print("q: ",  ['%.2f' % i for i in robot_states.q])
        print("theta: ", ['%.2f' % i for i in robot_states.theta])
        print("dq: ", ['%.2f' % i for i in robot_states.dq])
        print("dtheta: ", ['%.2f' % i for i in robot_states.dtheta])
        print("tau: ", ['%.2f' % i for i in robot_states.tau])
        print("tau_des: ", ['%.2f' % i for i in robot_states.tauDes])
        print("tau_dot: ", ['%.2f' % i for i in robot_states.tauDot])
        print("tau_ext: ", ['%.2f' % i for i in robot_states.tauExt])
        print("tcp_pose(quaternion): ", ['%.2f' % i for i in robot_states.tcpPose])
        print("tcp_pose_d: ", ['%.2f' % i for i in robot_states.tcpPoseDes])
        print("tcp_velocity: ", ['%.2f' % i for i in robot_states.tcpVel])
        print("camera_pose: ", ['%.2f' % i for i in robot_states.camPose])
        print("flange_pose: ", ['%.2f' % i for i in robot_states.flangePose])
        print("FT_sensor_raw_reading: ", ['%.2f' % i for i in robot_states.ftSensorRaw])
        print("F_ext_tcp_frame: ", ['%.2f' % i for i in robot_states.extWrenchInTcp])
        print("F_ext_base_frame: ", ['%.2f' % i for i in robot_states.extWrenchInBase])
        print("}")
        time.sleep(1)

def main():
    # Program Setup
    # ==============================================================================================
    # Parse arguments
    argparser = argparse.ArgumentParser()
    argparser.add_argument("robot_ip", help="IP address of the robot server")
    argparser.add_argument("local_ip", help="IP address of this PC")
    args = argparser.parse_args()

    # Instantiate Bestman_Real_Flexiv
    robot_controller = Bestman_Real_Flexiv(args.robot_ip, args.local_ip, frequency=1)

    # Print description
    print_description()

    try:
        # Initialize and clear faults
        robot_controller.Fault_clear()

        # Print States
        # =============================================================================
        # Thread for printing robot states
        print_thread = threading.Thread(target=print_robot_states, args=[robot_controller])
        print_thread.start()
        print_thread.join()

    except Exception as e:
        # Print exception error message
        robot_controller.log.error(str(e))

if __name__ == "__main__":
    main()