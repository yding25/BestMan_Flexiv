# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : display_states.py
# @Time           : 2024-11-30 21:05:25
# @Author         : Yan
# @Email          : yding25@binghamton.edu
# @Description    : Display arm state
# @Usage          : python display_states.py 192.168.2.100 192.168.2.108 10
"""

import argparse
import rospy
from Robotics_API import Bestman_Real_Flexiv

def print_test():
    """
    Function to run the 'test' printing task in a separate thread.
    """
    for _ in range(1000000):
        print("test")

def print_robot_states(robot_states):
    """
    Print robot states data @ 1Hz.
    """
    print("{")
    print("q: ", ["%.2f" % i for i in robot_states.q])
    print("theta: ", ["%.2f" % i for i in robot_states.theta])
    print("dq: ", ["%.2f" % i for i in robot_states.dq])
    print("dtheta: ", ["%.2f" % i for i in robot_states.dtheta])
    print("tau: ", ["%.2f" % i for i in robot_states.tau])
    print("tau_des: ", ["%.2f" % i for i in robot_states.tauDes])
    print("tau_dot: ", ["%.2f" % i for i in robot_states.tauDot])
    print("tau_ext: ", ["%.2f" % i for i in robot_states.tauExt])
    print("tcp_pose(quaternion): ", ["%.2f" % i for i in robot_states.tcpPose])
    print("tcp_pose_d: ", ["%.2f" % i for i in robot_states.tcpPoseDes])
    print("tcp_velocity: ", ["%.2f" % i for i in robot_states.tcpVel])
    print("camera_pose: ", ["%.2f" % i for i in robot_states.camPose])
    print("flange_pose: ", ["%.2f" % i for i in robot_states.flangePose])
    print("FT_sensor_raw_reading: ", ["%.2f" % i for i in robot_states.ftSensorRaw])
    print("F_ext_tcp_frame: ", ["%.2f" % i for i in robot_states.extWrenchInTcp])
    print("F_ext_base_frame: ", ["%.2f" % i for i in robot_states.extWrenchInBase])
    print("}" + "\n")


def main():
    # Parse Arguments
    argparser = argparse.ArgumentParser()
    argparser.add_argument("robot_ip", help="IP address of the robot server")
    argparser.add_argument("local_ip", help="IP address of this PC")
    argparser.add_argument(
        "frequency", help="command frequency, 1 to 200 [Hz]", type=int
    )
    args = argparser.parse_args()

    # Check if arguments are valid
    frequency = args.frequency
    assert 1 <= frequency <= 200, "Invalid <frequency> input"

    try:
        # Initialize ROS node
        rospy.init_node("robot_states_display", anonymous=True)

        # Instantiate robot interface
        bestman = Bestman_Real_Flexiv(args.robot_ip, args.local_ip, args.frequency)

        # Initialize robot
        if not bestman.initialize_robot():
            return  # Exit if initialization fails

        # Use ROS Timer to update and print robot states
        rospy.Timer(
            rospy.Duration(1.0 / frequency),
            lambda event: (
                bestman.update_robot_states(),
                print_robot_states(bestman.robot_states),
            ),
        )

        # Keep the program running
        rospy.spin()

    except Exception as e:
        # Print exception error message
        rospy.logerr(str(e))


if __name__ == "__main__":
    main()
