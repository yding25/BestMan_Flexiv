import numpy as np
import argparse
import time
import datetime
import ikpy
import serial
import serial.tools.list_ports
import minimalmodbus as mm
import pyRobotiqGripper
from ikpy.chain import Chain
from ikpy.inverse_kinematics import inverse_kinematic_optimization
from scipy.spatial.transform import Rotation as R
# Utility methods
from utility import quat2eulerZYX
from utility import parse_pt_states
from utility import list2str
# Import Flexiv RDK Python library
# fmt: off
import sys
sys.path.insert(0, "../lib_py")
import flexivrdk
# fmt: on


class Bestman_Real_Flexiv:
    def __init__(self, robot_ip, local_ip, frequency):
        self.robot = flexivrdk.Robot(robot_ip, local_ip)
        self.frequency = frequency
        self.log = flexivrdk.Log()
        self.mode = flexivrdk.Mode
        self.robot_states = flexivrdk.RobotStates()
        urdf_file = "../Asset/flexiv_rizon4_kinematics.urdf" #urdf_file path
        self.robot_chain = Chain.from_urdf_file(urdf_file)
        self.active_joints = [
            joint for joint in self.robot_chain.links 
            if isinstance(joint, ikpy.link.URDFLink) and (joint.joint_type == 'revolute' or joint.joint_type == 'prismatic')
        ]
        self.log_file = 'command_log.tum'


    def Prepare(self):
        # Parse Arguments
        argparser = argparse.ArgumentParser()
        # Required arguments
        argparser.add_argument("robot_ip", help="IP address of the robot server")
        argparser.add_argument("local_ip", help="IP address of this PC")
        argparser.add_argument(
            "frequency", help="command frequency, 1 to 200 [Hz]", type=int)
        # Optional arguments
        argparser.add_argument(
            "--hold", action="store_true",
            help="robot holds current joint positions, otherwise do a sine-sweep")
        args = argparser.parse_args()

        # Check if arguments are valid
        frequency = args.frequency
        assert (frequency >= 1 and frequency <= 200), "Invalid <frequency> input"

        self.log = flexivrdk.Log()

    def Fault_clear(self):
        # Clear fault on robot server if any
        if self.robot.isFault():
            self.log.warn("Fault occurred on robot server, trying to clear ...")
            # Try to clear the fault
            self.robot.clearFault()
            time.sleep(2)
            # Check again
            if self.robot.isFault():
                self.log.error("Fault cannot be cleared, exiting ...")
                return
            self.log.info("Fault on robot server is cleared")

        # Enable the robot, make sure the E-stop is released before enabling
        self.log.info("Enabling robot ...")
        self.robot.enable()

        # Wait for the robot to become operational
        seconds_waited = 0
        while not self.robot.isOperational():
            time.sleep(1)
            seconds_waited += 1
            if seconds_waited == 10:
                self.log.warn(
                    "Still waiting for robot to become operational, please "
                    "check that the robot 1) has no fault, 2) is booted "
                    "into Auto mode")

        self.log.info("Robot is now operational")

    def update_robot_states(self):
        """Updates the current robot states."""
        self.robot.getRobotStates(self.robot_states)


    def go_home(self):
        """Move arm to initial pose"""
        self.robot.setMode(self.mode.NRT_PRIMITIVE_EXECUTION)
        self.robot.executePrimitive("Home()")


    def pose_to_euler(self, pose):
        """
        Convert robot pose from a list [x, y, z, qw, qx, qy, qz] to [x, y, z] and Euler angles.
        
        Parameters:
        pose: list of 7 floats - [x, y, z, qw, qx, qy, qz]
        
        Returns:
        tuple: (x, y, z, roll, pitch, yaw) where (x, y, z) is the position and (roll, pitch, yaw) are the Euler angles in radians.
        """
        x, y, z, qw, qx, qy, qz = pose
        
        r = R.from_quat([qx, qy, qz, qw])  # Reordering to match scipy's [qx, qy, qz, qw]
        roll, pitch, yaw = r.as_euler('xyz', degrees=False)
        
        return [x, y, z, roll, pitch, yaw]


    def euler_to_pose(self, position_euler):
        """
        Convert robot pose from [x, y, z, roll, pitch, yaw] to [x, y, z, qw, qx, qy, qz].
        
        Parameters:
        position_euler: list of 6 floats - [x, y, z, roll, pitch, yaw]
        
        Returns:
        list: [x, y, z, qw, qx, qy, qz]
        """
        x, y, z, roll, pitch, yaw = position_euler
        
        r = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)
        qx, qy, qz, qw = r.as_quat()  # Getting [qx, qy, qz, qw] from scipy
        
        return [x, y, z, qw, qx, qy, qz]  # Reordering to match [qw, qx, qy, qz]
    

    def log_command(self, timestamp, target_pos):
        """record timestamp and target_pos"""
        with open(self.log_file, 'a') as file:
            file.write(f'{timestamp} {target_pos[0]} {target_pos[1]} {target_pos[2]} {target_pos[3]} {target_pos[4]} {target_pos[5]} {target_pos[6]} \n')


#==========================================================================================info

    def get_joint_bounds(self):
        """
        Retrieves the joint bounds of the robot arm.

        Returns:
            list: A list of tuples representing the joint bounds, where each tuple contains the minimum and maximum values for a joint.
        """
        maxbounds = self.robot.info().qMax
        minbounds = self.robot.info().qMin
        jointbounds = list(zip(maxbounds,minbounds))
        return jointbounds
    

    def print_joint_link_info(self, name):
        """
        Prints the joint and link information of a robot.

        Args:
            name (str): 'base' or 'arm'
        """
        if name == 'base':
            print("Base joint and link information:")
            for i, link in enumerate(self.robot_chain.links[:1]):  # Assuming the base is the first link
                print(f"Link {i}: {link.name}")
        elif name == 'arm':
            print("Arm joint and link information:")
            for i, link in enumerate(self.robot_chain.links[1:]):  # Assuming the arm starts from the second link
                print(f"Link {i + 1}: {link.name}")


    def get_arm_id(self):
        """
        Retrieves the ID of the robot arm.

        Returns:
            int: The serialNum of the robot arm.
        """
        return self.robot.info().serialNum


    def get_DOF(self):
        """
        Retrieves the degree of freedom (DOF) of the robot arm.

        Returns:
            int: The degree of freedom of the robot arm.
        """
        self.update_robot_states()
        return len(self.robot_states.q)
    

    def get_joint_idx(self):
        """
        Retrieves the indices of the joints in the robot arm.

        Returns:
            list: A list of indices for the joints in the robot arm.
        """
        return list(range(len(self.active_joints)))
    

    def get_tcp_link(self):
        """
        Retrieves the TCP (Tool Center Point) link of the robot arm.

        Returns:
            str: The TCP link of the robot arm.
        """
        return self.robot_chain.links[7].name


    def get_current_joint_values(self):
        """
        Retrieves the current joint angles of the robot arm.

        Returns:
            list: A list of the current joint angles of the robot arm.
        """
        self.update_robot_states()
        joint_values = self.robot_states.q
        return joint_values


#==========================================================================================joint_move

    def move_arm_to_joint_values(self, target_pos, target_vel=None, target_acc=None, MAX_VEL=None, MAX_ACC=None):
        """
        Move arm to a specific set of joint angles, considering physics.

        Args:
            target_pos: A list of desired joint angles (in radians) for each joint of the arm.
            target_vel: Optional. A list of target velocities for each joint.
            target_acc: Optional. A list of target accelerations for each joint.
            MAX_VEL: Optional. A list of maximum velocities for each joint.
            MAX_ACC: Optional. A list of maximum accelerations for each joint.
        """
        self.robot.setMode(self.mode.NRT_JOINT_POSITION)
        self.update_robot_states()
        DOF = len(self.robot_states.q)
        # Set default values if not provided
        if target_vel is None:
            target_vel = [0.0] * DOF
        if target_acc is None:
            target_acc = [0.0] * DOF
        if MAX_VEL is None:
            MAX_VEL = [1.0] * DOF
        if MAX_ACC is None:
            MAX_ACC = [0.5] * DOF
        
        self.robot.sendJointPosition(target_pos, target_vel, target_acc, MAX_VEL, MAX_ACC)
        time.sleep(1)


    def move_joint_traject(self, targets, target_vel=None, target_acc=None, MAX_VEL=None, MAX_ACC=None):
        """
        Move arm to a specific set of joint angles, considering physics.

        Args:
            target_pos: A list of desired joint angles (in radians) for each joint of the arm.
            target_vel: Optional. A list of target velocities for each joint.
            target_acc: Optional. A list of target accelerations for each joint.
            MAX_VEL: Optional. A list of maximum velocities for each joint.
            MAX_ACC: Optional. A list of maximum accelerations for each joint.
        """
        period = 1.0 / self.frequency
        self.robot.setMode(self.mode.NRT_JOINT_POSITION)
        self.update_robot_states()
        DOF = len(self.robot_states.q)
        if target_vel is None:
            target_vel = [0.0] * DOF
        if target_acc is None:
            target_acc = [0.0] * DOF
        if MAX_VEL is None:
            MAX_VEL = [3.0] * DOF
        if MAX_ACC is None:
            MAX_ACC = [1.0] * DOF

        for target_pos in targets:
            # Monitor fault on robot server
            if self.robot.isFault():
                raise Exception("Fault occurred on robot server, exiting ...")

            # Send command
            self.robot.sendJointPosition(target_pos, target_vel, target_acc, MAX_VEL, MAX_ACC)
            print(f"Sent joint positions: {target_pos}")

            # Use sleep to control loop period
            time.sleep(period)

#==============================================================================================effector_move

    def get_current_end_effector_pose(self):
        """
        Retrieves the current pose of the robot arm's end effector.

        This function obtains the position and orientation of the end effector.

        Returns:
            position: the [x, y, z] value of tcp
            orientation: [qw, qx, qy, qz] quaternion of tcp
        """

        self.update_robot_states()
        pose = self.robot_states.tcpPose
        position = pose[:3]
        orientation = pose[3:]
        return position, orientation


    def move_end_effector_to_goal_pose(self, end_effector_goal_pose, max_linear_vel=0.5, max_angular_vel=1.0):
        """
        Move arm's end effector to a target position.

        Args:
            end_effector_goal_pose (Pose): The desired pose of the end effector (includes both position and euler_orientation)
            max_linear_vel (float, optional): Maximum linear velocity. Defaults to 0.5.
            max_angular_vel (float, optional): Maximum angular velocity. Defaults to 1.0.
        """
        wrench = [0.0] * 6
        end_effector_goal_pose_que= self.euler_to_pose(end_effector_goal_pose)
        self.robot.setMode(self.mode.NRT_CARTESIAN_MOTION_FORCE)
        self.robot.sendCartesianMotionForce(end_effector_goal_pose_que, wrench, max_linear_vel, max_angular_vel)

    def move_end_effector_to_goal_pose_wrench(self, end_effector_goal_pose, wrench, max_linear_vel=0.5, max_angular_vel=1.0, contact_wrench=None, pressing_force_threshold=10.0):
        """
        Move arm's end effector to a target position.

        Args:
            end_effector_goal_pose (Pose): The desired pose of the end effector (includes both position and euler_orientation)
            wrench: Target TCP wrench (force and moment) in the force control reference frame, [fx, fy, fz, mx, my, mz] Unit: [N] [Nm]
            max_linear_vel (float, optional): Maximum linear velocity. Defaults to 0.5.
            max_angular_vel (float, optional): Maximum angular velocity. Defaults to 1.0.
            contact_wrench: Maximum contact wrench (force and moment) for contact detection, [fx, fy, fz, mx, my, mz]
        """
        end_effector_goal_pose_que = self.euler_to_pose(end_effector_goal_pose)
        self.robot.setMode(self.mode.NRT_CARTESIAN_MOTION_FORCE)

        # set max contact wrench
        if contact_wrench is not None:
            self.robot.setMaxContactWrench(contact_wrench)

        self.robot.sendCartesianMotionForce(end_effector_goal_pose_que, wrench, max_linear_vel, max_angular_vel)


    def move_effector_traject(self, targets, max_linear_vel=0.5, max_angular_vel=1.0):
        """
        Move arm's end effector to a target position.

        Args:
            targets: The desired pose of the end effector (includes both position and euler_orientation)
            max_linear_vel (float, optional): Maximum linear velocity. Defaults to 0.5.
            max_angular_vel (float, optional): Maximum angular velocity. Defaults to 1.0.
        """
        wrench = [0.0] * 6
        period = 1.0 / self.frequency
        self.robot.setMode(self.mode.NRT_JOINT_POSITION)
        self.update_robot_states()
        self.robot.setMode(self.mode.NRT_CARTESIAN_MOTION_FORCE)
        for target_pos in targets:
            # Monitor fault on robot server
            if self.robot.isFault():
                raise Exception("Fault occurred on robot server, exiting ...")

            self.robot.sendCartesianMotionForce(target_pos, wrench, max_linear_vel, max_angular_vel)
            # Use sleep to control loop period
            time.sleep(period)


    def move_end_effector_to_goal_position(self, end_effector_goal_position):
        """
        Move arm's end effector to a target position.

        Args:
            end_effector_goal_position: The desired pose of the end effector (includes both position and orientation).
        """
        pass


#==============================================================================================rotate_end_effector

    def rotate_end_effector_tcp(self, axis, angle):
        """
        Rotate the end effector of the robot arm by a specified angle by euler.

        Args:
            axis (str): Axis to rotate around ('x', 'y', or 'z').
            angle (float): The desired rotation angle in radians.
        """
        axis_index_map = {'x': 3, 'y': 4, 'z': 5}
        
        if axis not in axis_index_map:
            raise ValueError("Axis must be 'x', 'y', or 'z'.")

        axis_index = axis_index_map[axis]
        
        position, orientation = self.get_current_end_effector_pose()
        current_pose = position + orientation
        euler_angles = self.pose_to_euler(current_pose)
        euler_angles[axis_index] += angle
        new_pose = self.euler_to_pose(euler_angles)
        
        self.robot.setMode(self.mode.NRT_CARTESIAN_MOTION_FORCE_BASE)
        self.robot.sendCartesianMotionForce(new_pose)


    def rotate_end_effector_joint(self, angle):
        """
        Rotate the end effector of the robot arm by a specified angle by joint.

        Args:
            angle (float): The desired rotation angle in radians.
        """
        current_joint_values = self.get_current_joint_values()
        
        target_joint_values = current_joint_values.copy()
        target_joint_values[6] += angle 
        DOF = len(self.robot_states.q)
        target_vel = [0.0] * DOF
        target_acc = [0.0] * DOF
        MAX_VEL = [1.0] * DOF
        MAX_ACC = [1.0] * DOF

        self.robot.setMode(self.mode.NRT_JOINT_POSITION)
        self.robot.sendJointPosition(target_joint_values, target_vel, target_acc, MAX_VEL, MAX_ACC)
        

#==============================================================================================ik

    def joints_to_cartesian(self, joint_values):
        """
        Transforms the robot arm's joint angles to its Cartesian coordinates.

        Args:
            joint_values (list): A list of joint angles for the robot arm.

        Returns:
            tuple: A tuple containing the Cartesian coordinates (position and orientation) of the robot arm.
        """
        # Validate the number of joint values matches the number of active joints
        if len(joint_values) != len(self.active_joints):
            raise ValueError("The number of joint values does not match the number of active joints")
        
        # Map joint values to the full joint chain
        full_joint_values = np.zeros(len(self.robot_chain.links))
        active_joint_indices = [self.robot_chain.links.index(joint) for joint in self.active_joints]

        for i, joint_value in enumerate(joint_values):
            full_joint_values[active_joint_indices[i]] = joint_value

        # Calculate the end effector position and orientation
        cartesian_matrix = self.robot_chain.forward_kinematics(full_joint_values)

        # Extract position and orientation
        position = cartesian_matrix[:3, 3]
        orientation_matrix = cartesian_matrix[:3, :3]

        # Convert rotation matrix to quaternion
        r = R.from_matrix(orientation_matrix)
        quaternion = r.as_quat()
        orientation = [quaternion[3], quaternion[0], quaternion[1], quaternion[2]]

        return position, orientation
    
    
    def cartesian_to_joints(self, position, orientation):
        """
        Transforms the robot arm's Cartesian coordinates to its joint angles.

        Args:
            position (list): The Cartesian position of the robot arm.
            orientation (list): The Cartesian orientation of the robot arm.

        Returns:
            list: A list of joint angles corresponding to the given Cartesian coordinates.
        """
        rotation_matrix = R.from_euler('xyz', orientation).as_matrix()

        # Combine rotation matrix and position into a list
        target_pose = np.eye(4)
        target_pose[:3, :3] = rotation_matrix
        target_pose[:3, 3] = position

        initial_joint_angles = [0] * len(self.robot_chain)

        # inverse kinematics calculations and return joint angles
        joint_values = ikpy.inverse_kinematics.inverse_kinematic_optimization(
        chain=self.robot_chain,
        target_frame=target_pose,
        starting_nodes_angles=initial_joint_angles,
        orientation_mode='all',         
        )
     
        return joint_values[1:8]


    def calculate_IK_error(self, goal_position, goal_orientation):
        """
        Calculate the inverse kinematics (IK) error for performing pick-and-place manipulation of an object using a robot arm.

        Args:
            goal_position: The desired goal position for the target object.
            goal_orientation: The desired goal orientation for the target object.
        """
        pass
#==========================================================================================gripper
    
    def find_gripper(self):
    # Get all possible serial ports
        ports = [f'/dev/ttyS{i}' for i in range(2)] + [f'/dev/ttyUSB{i}' for i in range(2)]
        for port in ports:
            try:
                print(f"Trying port: {port}")
                # Try to open the serial port
                ser = serial.Serial(port, baudrate=115200, timeout=1.0)
                # Create Instrument object
                device = mm.Instrument(ser, 9, mode=mm.MODE_RTU)
                print(f"Connected to {port}")

                # Try sending read register command
                device.write_registers(1000, [0, 100, 0])
                registers = device.read_registers(2000, 3, 4)
                posRequestEchoReg3 = registers[1] & 0b0000000011111111

                # Check if it is a Gripper 
                if posRequestEchoReg3 == 100:
                    print(f"Gripper found on port: {port}")
                    ser.close()
                    return port
                ser.close()
            except Exception as e:
                print(f"Port {port} is not the gripper: {e}")
        return None
    
    
    def active_gripper(self, value, speed, force):
        """
        Retrieves the TCP (Tool Center Point) link of the robot arm.

        Returns:
            str: The TCP link of the robot arm.
        """

        gripper_port = self.find_gripper()
        if gripper_port:
            try:
                # Initialize the RobotiqGripper object using the detected port
                gripper = pyRobotiqGripper.RobotiqGripper(portname=gripper_port)
                print(f"Connected to gripper on port {gripper.portname}.")

                # activate gripper
                #print("Activating gripper...")
                #gripper.activate()

                # open gripper
                #print("Opening gripper...")
                #gripper.goTo(position=value, speed=speed, force=force)

                # close gripper
                print("Opening gripper...")
                gripper.goTo(position=40, speed=speed, force=force)

                # close gripper
                #print("Closing gripper...")
                #gripper.goTo(position=value, speed=speed, force=force)

            except Exception as e:
                print(f"Unexpected error: {e}")
        else:
            print("No gripper detected.")
