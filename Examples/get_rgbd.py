# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : get_rgbd.py
# @Time           : 2024-12-02 01:32:39
# @Author         : Yan
# @Email          : yding25@binghamton.edu
# @Description    : Get rgbd
# @Usage          : python get_rgbd.py 192.168.2.100 192.168.2.108 20
"""

import argparse
import rospy
from Robotics_API import Bestman_Real_Flexiv, Pose
from Sensor import Camera


def main():
    # Parse Arguments
    argparser = argparse.ArgumentParser(description="Move the robot arm to follow a trajectory.")
    # Required arguments
    argparser.add_argument("robot_ip", help="IP address of the robot server")
    argparser.add_argument("local_ip", help="IP address of this PC")
    argparser.add_argument("frequency", type=int, help="Command frequency, 1 to 200 [Hz]")
    args = argparser.parse_args()

    # Validate the frequency argument
    frequency = args.frequency
    assert 1 <= frequency <= 200, "Invalid <frequency> input"

    try:
        # Initialize ROS node
        rospy.init_node("robot_states_display", anonymous=True)

        # Instantiate the robot interface
        bestman = Bestman_Real_Flexiv(args.robot_ip, args.local_ip, args.frequency)

        # Initialize robot
        if not bestman.initialize_robot():
            return  # Exit if initialization fails

        # Define target pose
        target_pose = Pose([0.5628906488418579, -0.08013617247343063, 0.4745604693889618], [0.0014537398237735033, -0.039826150983572006, 0.9992029070854187, 0.0023102618288248777])
        bestman.move_eef_to_goal_pose(target_pose)
        
        # Wait for motion completion (This method will block the main threa)
        bestman.wait_for_motion_completion_eef(target_pose)
        
        # Marker detection and transformation
        print('start checking RGBD')
        camera = Camera(device_id='239722070506')
        
        # Show rgbd
        camera.display('rgbd')

    except Exception as e:
        # Log any exceptions that occur
        bestman.log.error(str(e))

if __name__ == "__main__":
    main()