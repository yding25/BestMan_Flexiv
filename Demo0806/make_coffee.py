'''
Run this script using:

python make_coffee.py 192.168.2.100 192.168.2.108 20
'''

import sys
import os
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(parent_dir, 'RoboticsToolBox'))
sys.path.append(os.path.join(parent_dir, 'Visualization'))
# Check sys.path
print('=' * 10 + "System Path" + '=' * 10)
for i, path in enumerate(sys.path):
    print(f"{i + 1}. {path}")
print('=' * 30)
from RoboticsToolBox.Bestman_flexiv import Bestman_Real_Flexiv
from RoboticsToolBox.utils import pose_to_euler, load_poses_from_xml
from Visualization.camera import Camera
import numpy as np
import argparse
import time


def main():
    # Parse Arguments
    argparser = argparse.ArgumentParser(description="Move the robot arm to follow a trajectory.")
    # Required arguments
    argparser.add_argument("robot_ip", help="IP address of the robot server")
    argparser.add_argument("local_ip", help="IP address of this PC")
    argparser.add_argument("frequency", type=int, help="Command frequency, 1 to 200 [Hz]")
    # Optional arguments
    argparser.add_argument("--hold", action="store_true", help="Robot holds current joint positions, otherwise do a sine-sweep")
    args = argparser.parse_args()

    # Validate the frequency argument
    frequency = args.frequency
    assert 1 <= frequency <= 200, "Invalid <frequency> input"

    # Instantiate the robot interface
    bestman = Bestman_Real_Flexiv(args.robot_ip, args.local_ip, args.frequency)
    
    try:
        # Clear fault on the robot server if any
        bestman.clear_fault()

        # Get and log current joint values and bounds
        joint_angles = bestman.get_current_joint_angles()
        bestman.log.info(f"Current joint angles: {joint_angles}")

        joint_bounds = bestman.get_joint_bounds()
        bestman.log.info(f"Current joint bounds: {joint_bounds}")

        # pose1 for being home 1
        pose1 = [0.5628906488418579, -0.08013617247343063, 0.4745604693889618, 0.0014537398237735033, -0.039826150983572006, 0.9992029070854187, 0.0023102618288248777]
        pose1 = pose_to_euler(pose1)
        bestman.move_end_effector_to_goal_pose(pose1)
        time.sleep(5)

        # pose2 for detecting marker
        pose2= [0.714484453201294, 0.015231587924063206, 0.5131312012672424, 0.033157266676425934, -0.04292983189225197, 0.9984701871871948, 0.01071979384869337]
        pose2 = pose_to_euler(pose2)
        bestman.move_end_effector_to_goal_pose(pose2)
        time.sleep(5)

        # Marker detection and transformation
        camera = Camera(device_id='239722070506')
        marker_positions = camera.get_marker_positions(debug=True)
        print(f'marker_positions:\n {marker_positions}')

        if marker_positions is not None:
            print(f'Marker positions in camera frame:\n {marker_positions}')
            
            # Get TCP pose
            bestman.update_robot_states()
            robot_states = bestman.robot_states
            tcp_pose = robot_states.tcpPose[:3]
            print(f'Get TCP pose:\n {tcp_pose}')
            
            # Transform marker positions to robot base frame
            pos_cam_world = tcp_pose + np.array([0.074719, 0, 0.148997 + 0.06])
            pos_marker_world = pos_cam_world - marker_positions
            print(f'Marker positions in robot base frame:\n {pos_marker_world}')
        else:
            print('Error: No markers detected.')

        # TODO: manully revise the marker pose
        pose_to_go = [pos_marker_world[0][0] - 0.055, pos_marker_world[0][1] + 0.04 , 0.36, 0.033157266676425934, -0.04292983189225197, 0.9984701871871948, 0.01071979384869337]
        print("pose to go is: ",pose_to_go)
       
        # activate gripper
        bestman.connect_gripper()
        time.sleep(1)
        bestman.close_gripper()
        time.sleep(1)

        # move to button, next press on it, and finally leave it
        target_trajectory = load_poses_from_xml(filename="recorded_traj/saved_poses.xml")
        bestman.move_end_effector_follow_trajectory(target_trajectory, max_linear_vel=0.1, max_angular_vel=0.5)
        time.sleep(3)
        
        # open gripper
        bestman.open_gripper()
        time.sleep(3)

        # move to cup, next ready for grasping cup, finally grasp cup
        target_trajectory2 = load_poses_from_xml(filename="recorded_traj/saved_poses2.xml")
        bestman.move_end_effector_follow_trajectory(target_trajectory2, max_linear_vel=0.1, max_angular_vel=0.5)
        time.sleep(1)
        bestman.close_gripper()
        time.sleep(1)
        
        # slightly leave the coffee maker
        target_trajectory3 = load_poses_from_xml(filename="recorded_traj/saved_poses3.xml")
        bestman.move_end_effector_follow_trajectory(target_trajectory3, max_linear_vel=0.1, max_angular_vel=0.5)
        time.sleep(1)
        
        # ready for putting cup on UAV
        bestman.move_end_effector_to_goal_pose(pose2)
        time.sleep(5)

        # move cup on top of UAV
        pose_to_go_top = [pos_marker_world[0][0] - 0.08, pos_marker_world[0][1] + 0.04 , 0.43, 0.033157266676425934, -0.04292983189225197, 0.9984701871871948, 0.01071979384869337]
        _pose_to_go_top = pose_to_euler(pose_to_go_top)
        bestman.move_end_effector_to_goal_pose(_pose_to_go_top)
        time.sleep(3)

        # put cup on UAV
        pose_togo = pose_to_euler(pose_to_go)
        bestman.move_end_effector_to_goal_pose(pose_togo)
        time.sleep(3)
        bestman.open_gripper()

        # pose1 for being home 2
        pose_stop = [0.328906488418579, -0.08013617247343063, 0.4745604693889618, 0.0014537398237735033, -0.039826150983572006, 0.9992029070854187, 0.0023102618288248777]
        _pose_stop = pose_to_euler(pose_stop)
        bestman.move_end_effector_to_goal_pose(_pose_stop)
        time.sleep(5)

    except Exception as e:
        # Log any exceptions that occur
        bestman.log.error(str(e))

if __name__ == "__main__":
    main()