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


def print_robot_states(robot_states):
    """
    Print robot states data.

    Parameters:
        robot_states: Object
            A data object containing the real-time state of the robot, which includes:
            - Joint positions (q)
            - Joint velocities (dq)
            - Torque readings, sensor data, and end-effector information.

    Each key printed corresponds to specific states of the robot:

    q : list of floats
        Joint positions [rad]. Describes the angular position of each joint.
        Example: ["0.00", "1.57", "-0.78"] for a 3-DOF robot.

    theta : list of floats
        Joint angles [rad]. Represents the same as 'q' in certain systems but may be used for normalized joint states.

    dq : list of floats
        Joint velocities [rad/s]. Indicates the rate of change of joint positions.

    dtheta : list of floats
        Joint angle velocities [rad/s]. Similar to 'dq' but for specific joint angle representations.

    tau : list of floats
        Measured joint torques [Nm]. Reflects the actual torque applied on each joint by the robot actuators.

    tau_des : list of floats
        Desired joint torques [Nm]. The torques the controller wants to apply to achieve the desired motion.

    tau_dot : list of floats
        Rate of change of joint torques [Nm/s]. Represents how fast the torque values are changing.

    tau_ext : list of floats
        External joint torques [Nm]. Estimates of torques caused by external forces applied on the robot.

    tcp_pose : list of floats
        Current TCP (Tool Center Point) position and orientation represented as a quaternion.
        Format: [x, y, z, qw, qx, qy, qz].
        - Position: x, y, z [m].
        - Orientation: qw, qx, qy, qz (unit quaternion).

    tcp_pose_d : list of floats
        Desired TCP position and orientation (target pose) in quaternion format.

    tcp_velocity : list of floats
        Velocity of the TCP in Cartesian space. Format: [vx, vy, vz, wx, wy, wz].
        - Linear velocity: vx, vy, vz [m/s].
        - Angular velocity: wx, wy, wz [rad/s].

    camera_pose : list of floats
        Pose of the robot's camera frame. Typically [x, y, z, qw, qx, qy, qz].

    flange_pose : list of floats
        Pose of the robot's flange (the mechanical interface before the tool). Format as TCP pose.

    FT_sensor_raw_reading : list of floats
        Raw readings from the force-torque (FT) sensor at the flange. Format: [Fx, Fy, Fz, Tx, Ty, Tz].
        - Forces: Fx, Fy, Fz [N].
        - Torques: Tx, Ty, Tz [Nm].

    F_ext_tcp_frame : list of floats
        External forces and torques in the TCP frame. Format: [Fx, Fy, Fz, Tx, Ty, Tz].

    F_ext_base_frame : list of floats
        External forces and torques transformed into the robot base frame.
    """
    print("{")
    print("q: ", ["%.2f" % i for i in robot_states.q]) # !
    print("theta: ", ["%.2f" % i for i in robot_states.theta])
    print("dq: ", ["%.2f" % i for i in robot_states.dq])
    print("dtheta: ", ["%.2f" % i for i in robot_states.dtheta])
    print("tau: ", ["%.2f" % i for i in robot_states.tau])
    print("tau_des: ", ["%.2f" % i for i in robot_states.tauDes])
    print("tau_dot: ", ["%.2f" % i for i in robot_states.tauDot])
    print("tau_ext: ", ["%.2f" % i for i in robot_states.tauExt])
    print("tcp_pose(quaternion): ", ["%.2f" % i for i in robot_states.tcpPose]) # !
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
