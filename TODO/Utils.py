# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : utils.py
# @Time           : 2024-12-02 01:01:41
# @Author         : Yan
# @Email          : yding25@binghamton.edu
# @Description    : Utils
# @Usage          : None
"""

from scipy.spatial.transform import Rotation as R
import xml.etree.ElementTree as ET

def quat2eulerZYX(quat, degree=False):
    """
    Convert a quaternion to Euler angles (ZYX order).

    Parameters
    ----------
    quat : list of float
        Quaternion in [w, x, y, z] order.
    degree : bool
        If True, return angles in degrees; otherwise, in radians.

    Returns
    -------
    list of float
        Euler angles in [x, y, z] order.
    """
    # Note: scipy's `R.from_quat` expects [x, y, z, w] order
    euler_angles = R.from_quat([quat[1], quat[2], quat[3], quat[0]]).as_euler(
        "xyz", degrees=degree
    )
    return euler_angles.tolist()


def print_description():
    """
    Print the description of the demonstration.
    """
    print("This script demonstrates teaching by demonstration for a robotic arm.")
    print("Free-drive the robot to record Cartesian poses and reproduce them.")
    print()


def list2str(lst):
    """
    Convert a list to a space-separated string.

    Parameters
    ----------
    lst : list
        List of elements to convert.

    Returns
    -------
    str
        Space-separated string representation of the list.
    """
    return " ".join(map(str, lst)) + " "


def parse_primitive_state(states, target_state):
    """
    Extract the value of a specific primitive state from a list of states.

    Parameters
    ----------
    states : list of str
        List of primitive state strings.
    target_state : str
        The state name to extract.

    Returns
    -------
    str
        Value of the target state, or an empty string if not found.
    """
    for state in states:
        words = state.split()
        if words[0] == target_state:
            return words[-1]
    return ""


def parse_pt_states(pt_states, parse_target):
    """
    Parse the value of a specified primitive state from the pt_states string list.

    Parameters
    ----------
    pt_states : str list
        Primitive states string list returned from Robot::getPrimitiveStates().
    parse_target : str
        Name of the primitive state to parse for.

    Returns
    ----------
    str
        Value of the specified primitive state in string format. Empty string is
        returned if parse_target does not exist.
    """
    for state in pt_states:
        # Split the state sentence into words
        words = state.split()

        if words[0] == parse_target:
            return words[-1]

    return ""


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
    for pose in root.findall("Pose"):
        position = pose.find("Position")
        orientation = pose.find("Orientation")
        pose_data = [
            float(position.get("x")),
            float(position.get("y")),
            float(position.get("z")),
            float(orientation.get("qx")),
            float(orientation.get("qy")),
            float(orientation.get("qz")),
            float(orientation.get("qw")),
        ]
        poses.append(pose_data)

    return poses


def pose_to_euler(pose):
    """
    Convert robot pose from a list [x, y, z, qw, qx, qy, qz] to [x, y, z] and Euler angles.

    Parameters:
    pose: list of 7 floats - [x, y, z, qw, qx, qy, qz]

    Returns:
    tuple: (x, y, z, roll, pitch, yaw) where (x, y, z) is the position and (roll, pitch, yaw) are the Euler angles in radians.
    """
    x, y, z, qw, qx, qy, qz = pose
    r = R.from_quat([qx, qy, qz, qw])  # Reordering to match scipy's [qx, qy, qz, qw]
    roll, pitch, yaw = r.as_euler("xyz", degrees=False)
    return [x, y, z, roll, pitch, yaw]


def euler_to_pose(position_euler):
    """
    Convert robot pose from [x, y, z, roll, pitch, yaw] to [x, y, z, qw, qx, qy, qz].

    Parameters:
    position_euler: list of 6 floats - [x, y, z, roll, pitch, yaw]

    Returns:
    list: [x, y, z, qw, qx, qy, qz]
    """
    x, y, z, roll, pitch, yaw = position_euler
    r = R.from_euler("xyz", [roll, pitch, yaw], degrees=False)
    qx, qy, qz, qw = r.as_quat()  # Getting [qx, qy, qz, qw] from scipy
    return [x, y, z, qw, qx, qy, qz]  # Reordering to match [qw, qx, qy, qz]
