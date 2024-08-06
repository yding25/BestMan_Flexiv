'''
Run this script using:

python move_arm_to_follow_trajectory.py 192.168.2.100 192.168.2.108 20
'''

import sys
import os
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(parent_dir, 'RoboticsToolBox'))
import pyRobotiqGripper
import xml.etree.ElementTree as ET
from Bestman_flexiv import *
from test_marker import get_marker_positions
import numpy as np

def load_poses_from_xml(filename="saved_poses.xml"):
    """
    Load poses from an XML file.

    Args:
        filename (str): Name of the XML file to load the poses.

    Returns:
        list: List of poses with positions and orientations.
    """
    tree = ET.parse(filename)
    root = tree.getroot()

    poses = []
    for pose in root.findall('Pose'):
        position = pose.find('Position')
        orientation = pose.find('Orientation')
        pose_data = [float(position.get('x')), float(position.get('y')), float(position.get('z')), float(orientation.get('qx')), float(orientation.get('qy')), float(orientation.get('qz')), float(orientation.get('qw'))]
        poses.append(pose_data)

    return poses




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

        # Get and log current joint values and bounds
        joint_angles = bestman.get_current_joint_angles()
        log.info(f"Current joint angles: {joint_angles}")

        joint_bounds = bestman.get_joint_bounds()
        log.info(f"Current joint bounds: {joint_bounds}")

        poses = load_poses_from_xml(filename="saved_poses.xml")
        target_trajectory = poses

        poses = load_poses_from_xml(filename="saved_poses2.xml")
        target_trajectory2 = poses

        poses = load_poses_from_xml(filename="saved_poses3.xml")
        target_trajectory3 = poses
        
        #poses = load_poses_from_xml(filename="saved_poses4.xml")
        #target_trajectory4 = poses


        pose1 = [0.5628906488418579, -0.08013617247343063, 0.4745604693889618, 0.0014537398237735033, -0.039826150983572006, 0.9992029070854187, 0.0023102618288248777]
        pose1 = bestman.pose_to_euler(pose1)

        pose2= [0.714484453201294, 0.015231587924063206, 0.5131312012672424, 0.033157266676425934, -0.04292983189225197, 0.9984701871871948, 0.01071979384869337]
        pose2 = bestman.pose_to_euler(pose2)

        bestman.move_end_effector_to_goal_pose(pose1)

        time.sleep(5)

        bestman.move_end_effector_to_goal_pose(pose2)
        time.sleep(5)
        # ! Marker
        marker_positions = get_marker_positions(debug=True)
        if marker_positions is not None:
            print("Marker positions in camera frame:\n", marker_positions)
            
            # Example TCP pose and camera-to-TCP transform
            tcp_pose = np.array([0.560, -0.081, 0.479])  # Replace with actual TCP pose

            bestman.update_robot_states()
            robot_states = bestman.robot_states
            
            tcp_pose = robot_states.tcpPose[:3]
            print("get tcp pose: ", tcp_pose)
            # camera_to_tcp_transform = np.array([-0.074719, 0, 0.148997 , 0, 0, 0])  # Replace with actual camera-to-TCP transform
            
            # Transform marker positions to robot base frame
            pos_cam_world = tcp_pose + np.array([0.074719,
                                                0,
                                                0.148997+0.06])

            # !
            pos_marker_world = pos_cam_world - marker_positions
            print("Marker positions in robot base frame:\n", pos_marker_world)
        else:
            print("No markers detected.")

        pose_to_go = [pos_marker_world[0][0] - 0.055, pos_marker_world[0][1] + 0.04 , 0.36, 0.033157266676425934, -0.04292983189225197, 0.9984701871871948, 0.01071979384869337]
        print("pose to go is: ",pose_to_go)
        # pose3 =  [0.68379273, 0.01081654, 0.23930327,0.033157266676425934, -0.04292983189225197, 0.9984701871871948, 0.01071979384869337]
        # pose3 = bestman.pose_to_euler(pose3)
        # bestman.move_end_effector_to_goal_pose(pose3)

        bestman.connect_gripper()
        time.sleep(1)
        bestman.close_gripper()
        time.sleep(1)



        bestman.move_end_effector_follow_trajectory(target_trajectory, max_linear_vel=0.1, max_angular_vel=0.5)
        time.sleep(3)

        bestman.open_gripper()
        time.sleep(3)
        bestman.move_end_effector_follow_trajectory(target_trajectory2, max_linear_vel=0.1, max_angular_vel=0.5)
        time.sleep(1)
        
        bestman.close_gripper()
        time.sleep(1)
        
        bestman.move_end_effector_follow_trajectory(target_trajectory3, max_linear_vel=0.1, max_angular_vel=0.5)
        time.sleep(1)
        
        bestman.move_end_effector_to_goal_pose(pose2)
        time.sleep(5)

        pose_to_go_top = [pos_marker_world[0][0] - 0.08, pos_marker_world[0][1] + 0.04 , 0.43, 0.033157266676425934, -0.04292983189225197, 0.9984701871871948, 0.01071979384869337]
        _pose_to_go_top = bestman.pose_to_euler(pose_to_go_top)
        bestman.move_end_effector_to_goal_pose(_pose_to_go_top)
        time.sleep(3)

        pose_togo = bestman.pose_to_euler(pose_to_go)
        bestman.move_end_effector_to_goal_pose(pose_togo)
        time.sleep(3)
        # bestman.move_end_effector_follow_trajectory(target_trajectory4, max_linear_vel=0.1, max_angular_vel=0.5)
        # time.sleep(1)
        
        bestman.open_gripper()


        pose_stop = [0.328906488418579, -0.08013617247343063, 0.4745604693889618, 0.0014537398237735033, -0.039826150983572006, 0.9992029070854187, 0.0023102618288248777]
        _pose_stop = bestman.pose_to_euler(pose_stop)
        bestman.move_end_effector_to_goal_pose(_pose_stop)
        time.sleep(5)



    except Exception as e:
        # Log any exceptions that occur
        log.error(str(e))


if __name__ == "__main__":
    main()
