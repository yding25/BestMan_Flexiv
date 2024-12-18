# python ik_for_flexiv.py 192.168.2.100 192.168.2.108 20


from ikpy.chain import Chain
import numpy as np
from scipy.spatial.transform import Rotation as R
from time import time


def calculate_new_pose(x, y, z, roll, pitch, yaw, distance=0.15):
    """
    基于给定的6D位姿 (x, y, z, roll, pitch, yaw),计算沿着z轴负方向移动后的新位姿。
    
    Args:
        x, y, z: 原始位置
        roll, pitch, yaw: 原始姿态 (欧拉角，弧度)
        distance: 沿着z负方向移动的距离,默认是0.15米
    
    Returns:
        new_x, new_y, new_z, roll, pitch, yaw: 新的6D位姿
    """
    # Step 1: 根据 roll, pitch, yaw 计算旋转矩阵
    rotation = R.from_euler('xyz', [roll, pitch, yaw])
    rotation_matrix = rotation.as_matrix()
    
    # Step 2: 提取旋转矩阵的 z 轴方向
    z_axis = rotation_matrix[:, 2]  # 第三列就是 z 轴方向
    
    # Step 3: 沿着 z 轴负方向移动 0.15
    new_position = np.array([x, y, z]) - distance * z_axis
    
    # Step 4: 返回新的位姿 (新位置 + 原来的姿态)
    return [new_position[0], new_position[1], new_position[2]], [roll, pitch, yaw]


# exit(0)
import sys
import os
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(parent_dir))
import pyRobotiqGripper
from RoboticsToolBox.Bestman_flexiv import *

from RoboticsToolBox.utils import tcp_pose_to_joint_angles
import argparse
def main():
    # Parse Arguments
    argparser = argparse.ArgumentParser(description="Move the robot arm to follow a trajectory.")
    # Required arguments
    argparser.add_argument("--robot_ip", help="IP address of the robot server", default="192.168.2.100")
    argparser.add_argument("--local_ip", help="IP address of this PC", default="192.168.2.108")
    argparser.add_argument("--frequency", type=int, help="Command frequency, 1 to 200 [Hz]", default=20)
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
        log.info(f"Current joint values: {joint_angles}")
        bestman.update_robot_states()
        tcp_pose = bestman.get_current_end_effector_pose()              #x y z roll pitch yaw
        tpos = [float(tcp) for tcp in tcp_pose]
        position, orientation= calculate_new_pose(tpos[0], tpos[1], tpos[2], tpos[3], tpos[4], tpos[5])           
        joint_angle = bestman.cartesian_to_joints(position, orientation)
        joint_angle = tcp_pose_to_joint_angles(tpos)
        bestman.move_arm_to_joint_angles(joint_angle)


        time.sleep(3)
        # log.info(f"joint_angle: {joint_angle}")         #IK             
        # log.info(f"joint_angle: {joint_angles}")        #GT
        # FK_tcp_pos, FK_tcp_ori = bestman.joints_to_cartesian(joint_angle)  #x y z qw qx qy qz
        # log.info(f"FK_tcp_pos: {FK_tcp_pos}")                               #x y z
        # log.info(f"tcp pose: {tpos}")                                       #
        # # log.info(f"FK_tcp_ori: {FK_tcp_ori}")                               #qw qx qy qz
        # # log.info(f"tcp ori: {ori}")                                         #qx qy qz qw
        
        # target_trajectory = [
        #     [0.5, 0, 0.3] + list(FK_tcp_ori)
        # ]
        # bestman.move_end_effector_follow_trajectory(target_trajectory, max_linear_vel=0.1, max_angular_vel=0.5)
        # time.sleep(2)

        # Define the target trajectory



    except Exception as e:
        # Log any exceptions that occur
        log.error(str(e))


if __name__ == "__main__":
    main()