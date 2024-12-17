# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : Bestman_Real_Flexiv.py
# @Time           : 2024-12-01 12:46:52
# @Author         : Yan
# @Email          : yding25@binghamton.edu
# @Description    : A basic class for Flexiv robotic arm
# @Usage          : None
"""


import os
import time
import numpy as np
from .Pose import Pose
from scipy.spatial.transform import Rotation as R
import ikpy
from ikpy.chain import Chain
from ikpy.link import URDFLink
import serial
import minimalmodbus as mm
import pyRobotiqGripper
import flexivrdk
import rospy


class Bestman_Real_Flexiv:
    """
    A class to interface with the Flexiv robotic arm.

    Attributes:
        robot: The Flexiv robot object initialized with provided IP addresses.
        gripper: The gripper object for controlling the robo's end effector.
        frequency: The control frequency for the robot.
        log: A logger object to log messages for debugging and monitoring.
        mode: A reference to the Flexiv operation modes.
        robot_states: A container for storing the current state of the robot.
        robot_chain: A kinematic chain for the robot, parsed from the URDF file.
        active_joints: A list of active (revolute or prismatic) joints in the robot.
        log_file: The file path for logging robot commands.
    """

    def __init__(self, robot_ip, local_ip, frequency):
        """
        Initializes the Flexiv robot and related components.

        Args:
            robot_ip (str): The IP address of the Flexiv robot.
            local_ip (str): The local IP address for communicating with the robot.
            frequency (float): Control frequency for the robot.
        """
        # Robot initialization
        self.robot = flexivrdk.Robot(robot_ip, local_ip)
        self.gripper = None
        self.frequency = frequency
        self.log = flexivrdk.Log()
        self.mode = flexivrdk.Mode
        self.robot_states = flexivrdk.RobotStates()

        # Parse URDF for kinematic chain
        current_dir = os.path.dirname(os.path.abspath(__file__))
        urdf_file = os.path.join(current_dir, "../Asset/flexiv_rizon4_kinematics.urdf")
        if not os.path.exists(urdf_file):
            raise FileNotFoundError(f"URDF file not found: {urdf_file}")
        # Parse the URDF file and configure the chain
        self.robot_chain = Chain.from_urdf_file(urdf_file)
        active_links_mask = (
            [False] + [True] * 7 + [False]
        )  # Base + 7 DOF + End Effector
        self.robot_chain.active_links_mask = active_links_mask
        # Filter active joints (revolute or prismatic)
        self.active_joints = [
            joint
            for joint in self.robot_chain.links
            if isinstance(joint, URDFLink)
            and (joint.joint_type == "revolute" or joint.joint_type == "prismatic")
        ]
        # Validate the number of active joints
        if len(self.active_joints) != 7:  # Flexiv is a 7-DOF robot
            raise ValueError(
                f"Expected 7 active joints, but found {len(self.active_joints)}. "
                f"Check the URDF file and active_links_mask."
            )
        # Log successful initialization
        self.log.info("Flexiv robot kinematic chain successfully initialized.")

        # Additional initializations can go here
        self.log_file = "command_log.tum"

    # ----------------------------------------------------------------
    # Device Initialization and State Management
    # ----------------------------------------------------------------

    def initialize_robot(self):
        """
        Initializes the robot by clearing faults and enabling it.

        This function ensures the robot is operational by:
        1. Clearing any existing faults.
        2. Enabling the robot.
        3. Waiting until the robot becomes operational.

        Returns:
            bool: True if the robot is successfully initialized, False otherwise.
        """
        try:
            # Clear faults
            if self.robot.isFault():
                self.log.warn("Fault occurred on robot server, attempting to clear...")
                self.robot.clearFault()
                time.sleep(1)
                if self.robot.isFault():
                    self.log.error("Fault cannot be cleared, exiting...")
                    return False
                self.log.info("Fault cleared successfully.")

            # Enable the robot
            self.log.info("Enabling the robot...")
            self.robot.enable()

            # Wait for the robot to become operational
            for seconds_waited in range(10):
                if self.robot.isOperational():
                    self.log.info("Robot is now operational.")
                    return True
                time.sleep(1)

            self.log.warn("Robot is not operational after 10 seconds.")
            return False

        except Exception as e:
            self.log.error(f"Failed to initialize the robot: {str(e)}")
            return False

    def clear_fault(self):
        """
        Clears any faults on the robot server and ensures the robot is operational.
        """
        if self.robot.isFault():
            self.log.warn("Fault detected. Attempting to clear...")
            self.robot.clearFault()
            time.sleep(2)
            if self.robot.isFault():
                self.log.error("Fault could not be cleared.")
                return
            self.log.info("Fault cleared successfully.")

        self.log.info("Enabling the robot...")
        self.robot.enable()

        for seconds_waited in range(10):
            if self.robot.isOperational():
                self.log.info("Robot is now operational.")
                return
            time.sleep(1)

        self.log.warn("Robot is not operational after 10 seconds.")

    def update_robot_states(self):
        """
        Updates the current robot states by fetching them from the robot.
        """
        self.robot.getRobotStates(self.robot_states)

    def go_home(self):
        """
        Moves the robot arm to its initial (home) pose.
        """
        self.robot.setMode(self.mode.NRT_PRIMITIVE_EXECUTION)
        self.robot.executePrimitive("Home()")

    # ----------------------------------------------------------------
    # Logging
    # ----------------------------------------------------------------

    def log_command(self, timestamp, target_pos):
        """
        Records a command with timestamp and target position.

        Args:
            timestamp (float): The time of the command.
            target_pos (list): Target joint positions or end effector pose.
        """
        try:
            with open(self.log_file, "a") as file:
                file.write(f"{timestamp} {' '.join(map(str, target_pos))}\n")
            self.log.info(f"Command logged: {timestamp}, {target_pos}")
        except Exception as e:
            self.log.error(f"Failed to log command: {str(e)}")

    # ----------------------------------------------------------------
    # Arm Functions
    # ----------------------------------------------------------------

    def get_joint_bounds(self):
        """
        Retrieves the joint limits of the robot arm.

        Returns:
            list: A list of tuples, where each tuple contains the minimum and maximum value for a joint.
        """
        max_bounds = self.robot.info().qMax
        min_bounds = self.robot.info().qMin
        return list(zip(min_bounds, max_bounds))

    def print_arm_jointInfo(self):
        """
        Prints the joint and link information of the robot's arm.

        Returns:
            None
        """
        print("Arm joint and link information:")
        for i, link in enumerate(
            self.robot_chain.links[1:]
        ):  # Assuming arm starts at the second link
            print(f"Link {i + 1}: {link.name}")
            self.log.info(f"Link {i + 1}: {link.name}")

    def get_arm_id(self):
        """
        Retrieves the serial ID of the robot arm.

        Returns:
            str: The serial number of the robot arm.
        """
        try:
            arm_id = self.robot.info().serialNum
            self.log.info(f"Retrieved robot arm ID: {arm_id}")
            return arm_id
        except Exception as e:
            self.log.error(f"Failed to get robot arm ID: {str(e)}")
            return None

    def get_DOF(self):
        """
        Retrieves the degree of freedom (DOF) of the robot arm.

        Returns:
            int: The number of degrees of freedom.
        """
        try:
            self.update_robot_states()
            dof = len(self.robot_states.q)
            self.log.info(f"Robot DOF: {dof}")
            return dof
        except Exception as e:
            self.log.error(f"Failed to retrieve DOF: {str(e)}")
            return 0

    def get_arm_all_joint_idx(self):
        """
        Retrieves the indices of all active joints in the robot arm.

        Returns:
            list[int]: A list of indices for the active joints.
        """
        joint_indices = list(range(len(self.active_joints)))
        self.log.info(f"Retrieved joint indices: {joint_indices}")
        return joint_indices

    def get_tcp_link(self):
        """
        Retrieves the name of the TCP (Tool Center Point) link.

        Returns:
            str: Name of the TCP link.
        """
        try:
            tcp_link_name = self.robot_chain.links[7].name  # Assuming TCP is at index 7
            self.log.info(f"TCP link retrieved: {tcp_link_name}")
            return tcp_link_name
        except IndexError as e:
            self.log.error(f"Failed to retrieve TCP link: {str(e)}")
            return None

    def get_current_joint_values(self):
        """
        Retrieves the current joint angles of the robot arm.

        Returns:
            list[float]: A list of current joint angles (in radians).
        """
        try:
            self.update_robot_states()
            joint_values = self.robot_states.q
            self.log.info(f"Current joint values: {joint_values}")
            return joint_values
        except Exception as e:
            self.log.error(f"Failed to retrieve current joint values: {str(e)}")
            return []

    def get_current_eef_pose(self):
        """
        Retrieves the current pose of the robot arm's end effector.

        Returns:
            Pose: A Pose object representing the end effector's position and orientation.
        """
        try:
            self.update_robot_states()
            tcp_pose = self.robot_states.tcpPose  # [x, y, z, qw, qx, qy, qz]
            position = tcp_pose[:3]
            orientation = tcp_pose[3:]  # [qw, qx, qy, qz]
            pose = Pose(position, orientation)
            self.log.info(f"Current end effector pose: {pose}")
            return pose
        except Exception as e:
            self.log.error(f"Failed to retrieve end effector pose: {str(e)}")
            return None

    def print_robot_info(self):
        """
        Print Flexiv robot information to confirm available attributes.
        """
        try:
            info = self.robot.info()
            print("Robot Info Attributes:")
            for attr in dir(info):
                if not attr.startswith("_"):
                    print(f"{attr}: {getattr(info, attr)}")
        except Exception as e:
            self.log.error(f"Failed to retrieve robot information: {str(e)}")

    def move_arm_to_joint_values(
        self, joint_values, target_vel=None, target_acc=None, MAX_VEL=None, MAX_ACC=None
    ):
        """
        Moves the robot arm to the specified joint angles with optional velocity and acceleration.

        Args:
            joint_values (list[float]): Desired joint angles (in radians) for each joint.
            target_vel (list[float], optional): Target velocities for each joint. Defaults to None.
            target_acc (list[float], optional): Target accelerations for each joint. Defaults to None.
            MAX_VEL (list[float], optional): Maximum velocities for each joint. Defaults to dqMax from RobotInfo.
            MAX_ACC (list[float], optional): Maximum accelerations for each joint. Defaults to [5.0] rad/s².

        Returns:
            None
        """
        try:
            # Set control mode
            self.robot.setMode(self.mode.NRT_JOINT_POSITION)
            self.update_robot_states()
            DOF = len(self.robot_states.q)

            # Get limits from RobotInfo
            alpha = 0.1  # Key coefficient to regulate the maximum speed, with a range of [0.0001, 1]
            robot_info = self.robot.info()
            default_max_vel = [vel * alpha for vel in robot_info.dqMax]
            default_max_acc = [
                5.0
            ] * DOF  # Assuming 5 rad/s² as default maximum acceleration

            # Set default values
            MAX_VEL = MAX_VEL or default_max_vel
            MAX_ACC = MAX_ACC or default_max_acc
            target_vel = target_vel if target_vel is not None else [0.0] * DOF
            target_acc = target_acc if target_acc is not None else [0.0] * DOF

            # Validate joint values within limits
            joint_limits = zip(robot_info.qMin, robot_info.qMax)
            for i, (q, (q_min, q_max)) in enumerate(zip(joint_values, joint_limits)):
                if not (q_min <= q <= q_max):
                    raise ValueError(
                        f"Joint {i + 1} value {q} out of range [{q_min}, {q_max}]"
                    )

            # Send joint command
            self.robot.sendJointPosition(
                joint_values, target_vel, target_acc, MAX_VEL, MAX_ACC
            )
            self.log.info(f"Moved arm to joint values: {joint_values}")
        except Exception as e:
            self.log.error(f"Failed to move arm to joint values: {str(e)}")

    # ----------------------------------------------------------------
    # End Effector (EEF) Functions
    # ----------------------------------------------------------------

    def move_eef_to_goal_pose(self, goal_pose, max_linear_vel=0.1, max_angular_vel=0.5):
        """
        Moves the end effector to the specified pose.

        Args:
            goal_pose (Pose): The desired pose (position and orientation in quaternion format).
            max_linear_vel (float): Maximum linear velocity. Defaults to 0.1 m/s.
            max_angular_vel (float): Maximum angular velocity. Defaults to 0.5 rad/s.

        Returns:
            None
        """
        try:
            if not isinstance(goal_pose, Pose):
                raise TypeError("pose must be an instance of Pose")

            wrench = [0.0] * 6  # No wrench force
            position = goal_pose.get_position()
            orientation = goal_pose.get_orientation(type="quaternion")
            pose = position + orientation

            self.robot.setMode(self.mode.NRT_CARTESIAN_MOTION_FORCE)
            self.robot.sendCartesianMotionForce(
                pose, wrench, max_linear_vel, max_angular_vel
            )
            self.log.info(f"Moved end effector to pose: {pose}")
        except Exception as e:
            self.log.error(f"Failed to move end effector to goal pose: {str(e)}")

    def move_eef_to_goal_pose_wrench(
        self,
        goal_pose,
        wrench,
        max_linear_vel=0.1,
        max_angular_vel=0.5,
        contact_wrench=None,
    ):
        """
        Moves the end effector to a target pose with a specified wrench control.

        Args:
            goal_pose (Pose): Desired pose of the end effector (position and quaternion orientation).
            wrench (list[float]): Target TCP wrench [fx, fy, fz, mx, my, mz] in the force control reference frame.
            max_linear_vel (float): Maximum linear velocity. Defaults to 0.1 m/s.
            max_angular_vel (float): Maximum angular velocity. Defaults to 0.5 rad/s.
            contact_wrench (list[float], optional): Maximum contact wrench for collision detection.

        Returns:
            None
        """
        try:
            if not isinstance(goal_pose, Pose):
                raise TypeError("goal_pose must be an instance of Pose")

            position = goal_pose.get_position()
            orientation = goal_pose.get_orientation(type="quaternion")
            pose = position + orientation  # Combine position and orientation

            self.robot.setMode(self.mode.NRT_CARTESIAN_MOTION_FORCE)

            if contact_wrench:
                self.robot.setMaxContactWrench(contact_wrench)
                self.log.info(f"Set contact wrench limit: {contact_wrench}")

            self.robot.sendCartesianMotionForce(
                pose, wrench, max_linear_vel, max_angular_vel
            )
            self.log.info(f"Moved EEF to pose: {goal_pose} with wrench: {wrench}")
        except Exception as e:
            self.log.error(f"Failed to move EEF to goal pose with wrench: {str(e)}")

    def rotate_eef_tcp(self, axis, angle):
        """
        Rotates the end effector TCP around a specified axis by a given angle.

        Args:
            axis (str): Axis to rotate around ('x', 'y', or 'z').
            angle (float): Rotation angle in radians.

        Returns:
            None
        """
        # TODO
        try:
            current_pose = self.get_current_eef_pose()
            position = current_pose.get_position()
            quaternion = current_pose.get_orientation(type="quaternion")

            rotation = R.from_quat(quaternion)
            if axis == "x":
                rotation = rotation * R.from_rotvec([angle, 0, 0])
            elif axis == "y":
                rotation = rotation * R.from_rotvec([0, angle, 0])
            elif axis == "z":
                rotation = rotation * R.from_rotvec([0, 0, angle])
            else:
                raise ValueError("Invalid axis. Axis must be one of 'x', 'y', or 'z'.")

            new_quaternion = list(rotation.as_quat())
            new_pose = Pose(position, new_quaternion)

            self.move_eef_to_goal_pose(new_pose)
            self.log.info(f"Rotated EEF TCP around {axis}-axis by {angle} radians.")
        except Exception as e:
            self.log.error(f"Failed to rotate EEF TCP: {str(e)}")

    def rotate_eef_joint(self, angle):
        """
        Rotates the end effector of the robot arm using the last joint.

        Args:
            angle (float): Desired rotation angle in radians.

        Returns:
            None
        """
        # TODO
        try:
            current_joint_values = self.get_current_joint_values()
            target_joint_values = current_joint_values.copy()

            if len(target_joint_values) <= 6:
                raise ValueError(
                    "Joint array is too short to perform EEF joint rotation."
                )

            target_joint_values[6] += angle  # Assuming the 7th joint rotates the EEF
            DOF = len(self.robot_states.q)
            target_vel = [0.0] * DOF
            target_acc = [0.0] * DOF
            MAX_VEL = [1.0] * DOF
            MAX_ACC = [1.0] * DOF

            self.robot.setMode(self.mode.NRT_JOINT_POSITION)
            self.robot.sendJointPosition(
                target_joint_values, target_vel, target_acc, MAX_VEL, MAX_ACC
            )
            self.log.info(f"Rotated EEF joint by {angle} radians.")
        except Exception as e:
            self.log.error(f"Failed to rotate EEF joint: {str(e)}")

    # ----------------------------------------------------------------
    # Functions for IK
    # ----------------------------------------------------------------

    def joints_to_cartesian(self, joint_values):
        """
        Converts the robot's joint angles to its Cartesian coordinates.

        Args:
            joint_values (list[float]): A list of joint angles for the robot arm.

        Returns:
            Pose: A Pose object representing the Cartesian position and quaternion orientation.

        Raises:
            ValueError: If the number of joint values does not match the number of active joints.
        """
        try:
            if len(joint_values) != len(self.active_joints):
                raise ValueError(
                    "The number of joint values does not match the number of active joints."
                )

            full_joint_values = np.zeros(len(self.robot_chain.links))
            active_joint_indices = [
                self.robot_chain.links.index(joint) for joint in self.active_joints
            ]

            for i, joint_value in enumerate(joint_values):
                full_joint_values[active_joint_indices[i]] = joint_value

            cartesian_matrix = self.robot_chain.forward_kinematics(full_joint_values)
            position = cartesian_matrix[:3, 3]
            orientation_matrix = cartesian_matrix[:3, :3]
            quaternion = R.from_matrix(orientation_matrix).as_quat()

            self.log.info(f"Converted joint values {joint_values} to Cartesian Pose.")
            return Pose(position, quaternion)
        except Exception as e:
            self.log.error(f"Error converting joint values to Cartesian: {str(e)}")
            raise

    def cartesian_to_joints(self, position, orientation):
        """
        Converts the robot's Cartesian coordinates to its joint angles.

        Args:
            position (list[float]): Cartesian position of the robot arm.
            orientation (list[float]): Quaternion orientation of the robot arm.

        Returns:
            list[float]: Joint angles corresponding to the given Cartesian coordinates.

        Raises:
            ValueError: If the solution is invalid or out of bounds.
        """
        try:
            rotation_matrix = R.from_quat(orientation).as_matrix()

            target_pose = np.eye(4)
            target_pose[:3, :3] = rotation_matrix
            target_pose[:3, 3] = position

            initial_joint_values = [0] * len(self.robot_chain)
            joint_values = ikpy.inverse_kinematics.inverse_kinematic_optimization(
                chain=self.robot_chain,
                target_frame=target_pose,
                starting_nodes_angles=initial_joint_values,
                orientation_mode="all",
            )

            if not self._validate_joint_limits(joint_values):
                raise ValueError("Joint values exceed physical joint limits.")

            self.log.info(f"Converted Cartesian position {position} to joint values.")
            return joint_values[1:8]
        except Exception as e:
            self.log.error(f"Error converting Cartesian to joint values: {str(e)}")
            raise

    def _validate_joint_limits(self, joint_values):
        """
        Validates if the joint values are within the robot's physical joint limits.

        Args:
            joint_values (list[float]): Joint values to validate.

        Returns:
            bool: True if all joint values are within limits, False otherwise.
        """
        joint_bounds = self.get_joint_bounds()
        for joint_value, (lower, upper) in zip(joint_values, joint_bounds):
            if not (lower <= joint_value <= upper):
                return False
        return True

    def calculate_IK_error(self, goal_position, goal_orientation):
        """
        Calculates the inverse kinematics (IK) error for reaching a target pose.

        Args:
            goal_position (list[float]): Desired Cartesian position of the robot arm.
            goal_orientation (list[float]): Desired quaternion orientation of the robot arm.

        Returns:
            float: The IK error as the Euclidean distance between the desired and actual pose.
        """
        try:
            joint_values = self.cartesian_to_joints(goal_position, goal_orientation)
            actual_pose = self.joints_to_cartesian(joint_values)

            position_error = np.linalg.norm(
                np.array(goal_position) - np.array(actual_pose.get_position())
            )
            orientation_error = np.linalg.norm(
                np.array(goal_orientation)
                - np.array(actual_pose.get_orientation(type="quaternion"))
            )

            total_error = position_error + orientation_error
            self.log.info(
                f"Calculated IK error. Position error: {position_error}, Orientation error: {orientation_error}."
            )
            return total_error
        except Exception as e:
            self.log.error(f"Failed to calculate IK error: {str(e)}")
            raise

    # ----------------------------------------------------------------
    # Gripper Functions
    # ----------------------------------------------------------------

    def find_gripper(self):
        """
        Searches for the gripper on available serial ports and returns the port if found.

        Returns:
            str: The serial port where the gripper is connected, or None if not found.
        """
        ports = [f"/dev/ttyS{i}" for i in range(2)] + [
            f"/dev/ttyUSB{i}" for i in range(2)
        ]

        for port in ports:
            try:
                print(f"Trying port: {port}")
                ser = serial.Serial(port, baudrate=115200, timeout=1.0)
                device = mm.Instrument(ser, 9, mode=mm.MODE_RTU)
                device.write_registers(1000, [0, 100, 0])
                registers = device.read_registers(2000, 3, 4)
                posRequestEchoReg3 = registers[1] & 0b0000000011111111

                if posRequestEchoReg3 == 100:
                    print(f"Gripper found on port: {port}")
                    return port
            except Exception as e:
                print(f"Port {port} is not the gripper: {e}")
            finally:
                # Ensure the serial port is closed after each attempt
                if "ser" in locals() and ser.is_open:
                    ser.close()

        return None

    def connect_gripper(self):
        """
        Connects to the gripper and activates it if necessary.
        """
        try:
            gripper_port = self.find_gripper()
            if gripper_port:
                self.gripper = pyRobotiqGripper.RobotiqGripper(portname=gripper_port)
                self.log.info(f"Connected to gripper on port {self.gripper.portname}.")

                if not self.gripper.isActivated():
                    self.log.info("Activating gripper...")
                    self.gripper.activate()
            else:
                self.log.error("Failed to connect to the gripper. No port detected.")
        except Exception as e:
            self.log.error(f"Failed to connect to gripper: {str(e)}")

    def gripper_goto(self, value, speed=255, force=255):
        """
        Moves the gripper to a specified position with given speed and force.

        Args:
            value (int): Position of the gripper (0: open, 255: closed).
            speed (int): Speed of the gripper movement (0-255).
            force (int): Force applied by the gripper (0-255).
        """
        try:
            if self.gripper:
                self.gripper.goTo(position=value, speed=speed, force=force)
                self.log.info(
                    f"Gripper moved to position {value} with speed {speed} and force {force}."
                )
            else:
                self.log.error("Gripper not connected. Call connect_gripper() first.")
        except Exception as e:
            self.log.error(f"Failed to move gripper: {str(e)}")

    def open_gripper(self, speed=255, force=255):
        """
        Opens the gripper to its maximum position with specified speed and force.

        Args:
            speed (int): Speed of the gripper movement (0-255). Default is 255.
            force (int): Force applied by the gripper (0-255). Default is 255.
        """
        try:
            self.gripper_goto(value=0, speed=speed, force=force)
            self.log.info(f"Opened gripper with speed={speed} and force={force}.")
        except Exception as e:
            self.log.error(f"Failed to open gripper: {str(e)}")

    def close_gripper(self, speed=255, force=255):
        """
        Closes the gripper to its minimum position with specified speed and force.

        Args:
            speed (int): Speed of the gripper movement (0-255). Default is 255.
            force (int): Force applied by the gripper (0-255). Default is 255.
        """
        try:
            self.gripper_goto(value=255, speed=speed, force=force)
            self.log.info(f"Closed gripper with speed={speed} and force={force}.")
        except Exception as e:
            self.log.error(f"Failed to close gripper: {str(e)}")

    # ----------------------------------------------------------------
    # Other Functions
    # ----------------------------------------------------------------
    # def wait_for_motion_completion(self, target_joint_values, error_threshold=0.01, speed_threshold=0.01, timeout=10):
    #     """
    #     Waits until the robot's motion is completed by checking the error between current and target joint values
    #     and monitoring joint speeds.

    #     Args:
    #         target_joint_values (list[float]): The target joint values (in radians).
    #         error_threshold (float): The allowable error threshold for each joint (in radians). Default is 0.01 rad.
    #         speed_threshold (float): The allowable speed threshold for each joint (in rad/s). Default is 0.01 rad/s.
    #         timeout (int): The maximum time to wait for completion (in seconds). Default is 10 seconds.

    #     Returns:
    #         bool: True if motion is completed within timeout, False otherwise.
    #     """
    #     motion_complete = {"status": False}
    #     start_time = rospy.get_time()

    #     def check_motion(event):
    #         self.update_robot_states()
    #         current_joint_values = self.robot_states.q
    #         current_joint_speeds = self.robot_states.dq
    #         # Calculate the absolute difference between current and target joint values
    #         errors = [abs(c - t) for c, t in zip(current_joint_values, target_joint_values)]
    #         # Check if motion is completed
    #         if all(error <= error_threshold for error in errors) and all(abs(speed) <= speed_threshold for speed in current_joint_speeds):
    #             motion_complete["status"] = True
    #             rospy.loginfo("Motion completed: Joint values and speeds are within thresholds.")
    #             rospy.signal_shutdown("Motion completed")
    #         if rospy.get_time() - start_time > timeout:
    #             rospy.logerr("Timeout: Motion did not complete within the specified time.")
    #             rospy.signal_shutdown("Timeout")

    #     # Start ROS Timer for checking motion status
    #     rospy.Timer(rospy.Duration(0.01), check_motion)  # Check every 10ms
    #     rospy.spin()

    #     return motion_complete["status"]

    def wait_for_joints(
        self,
        target_joint_values,
        error_threshold=0.01,
        speed_threshold=0.01,
        timeout=5,
    ):
        """
        Waits until the robot's motion is completed by checking the error between current and target joint values
        and monitoring joint speeds.

        Args:
            target_joint_values (list[float]): The target joint values (in radians).
            error_threshold (float): The allowable error threshold for each joint (in radians). Default is 0.01 rad.
            speed_threshold (float): The allowable speed threshold for each joint (in rad/s). Default is 0.01 rad/s.
            timeout (int): The maximum time to wait for completion (in seconds). Default is 10 seconds.

        Returns:
            bool: True if motion is completed within timeout, False otherwise.
        """
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < timeout:
            self.update_robot_states()
            current_joint_values = self.robot_states.q
            current_joint_speeds = self.robot_states.dq

            errors = [
                abs(c - t) for c, t in zip(current_joint_values, target_joint_values)
            ]
            if all(error <= error_threshold for error in errors) and all(
                abs(speed) <= speed_threshold for speed in current_joint_speeds
            ):
                rospy.loginfo("Motion completed successfully.")
                return True  # Exit on success

            rospy.sleep(0.002)  # Check every 5ms

        rospy.logerr("Timeout reached.")
        return False  # Timeout occurred

    def wait_for_eef(
        self,
        target_tcp_pose,
        position_error_threshold=0.03,
        timeout=5,
    ):
        """
        Waits until the robot's motion is completed by checking the error between current and target tcp pose
        and monitoring tcp velocities.

        Args:
            target_tcp_pose (Pose): The target tcp pose (position and orientation in quaternion format).
            position_error_threshold (float): The allowable position error threshold (in meters). Default is 0.01 m.
            orientation_error_threshold (float): The allowable orientation error threshold (in radians). Default is 0.01 rad.
            velocity_threshold (float): The allowable velocity threshold for the tcp (in m/s for linear and rad/s for angular). Default is 0.01 rad/s.
            timeout (int): The maximum time to wait for completion (in seconds). Default is 10 seconds.

        Returns:
            bool: True if motion is completed within timeout, False otherwise.
        """
        position = target_tcp_pose.get_position()
        orientation = target_tcp_pose.get_orientation(type="quaternion")
        target_tcp_pose = position + orientation  # Combine position and orientation

        print(f"target_tcp_pose:{target_tcp_pose}")

        start_time = rospy.get_time()
        while rospy.get_time() - start_time < timeout:
            self.update_robot_states()
            current_tcp_pose = self.robot_states.tcpPose  # Get the current TCP pose
            # current_tcp_velocity = (
            #     self.robot_states.tcpVel
            # )  # Get the current TCP velocity

            # Compute position error (Euclidean distance between position vectors)
            position_error = np.linalg.norm(
                np.array(current_tcp_pose[:3]) - np.array(target_tcp_pose[:3])
            )

            # Check if position errors are within threshold and velocities are below threshold
            if (
                position_error <= position_error_threshold
            ):  # and np.linalg.norm(current_tcp_velocity[:3]) <= velocity_threshold
                rospy.loginfo("Motion completed successfully.")
                return True  # Exit on success

            rospy.sleep(0.01)  # Check every 10ms

        rospy.logerr("Timeout reached.")
        return False  # Timeout occurred

    def compute_orientation_error(self, current_orientation, target_orientation):
        """
        Compute the orientation error between two quaternions.
        """
        # Convert quaternions to rotation matrices or use quaternion distance
        q_current = np.array(current_orientation)
        q_target = np.array(target_orientation)

        # Compute quaternion distance (angle)
        dot_product = np.dot(q_current, q_target)
        angle = 2 * np.arccos(
            np.clip(dot_product, -1.0, 1.0)
        )  # Returns the angle in radians
        return angle
