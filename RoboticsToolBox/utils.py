#!/usr/bin/env python

"""utility.py

Utility methods.
"""

__copyright__ = "Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import math
from scipy.spatial.transform import Rotation as R
import xml.etree.ElementTree as ET

def quat2eulerZYX(quat, degree=False):
    """
    Convert quaternion to Euler angles with ZYX axis rotations.

    Parameters
    ----------
    quat : float list
        Quaternion input in [w,x,y,z] order.
    degree : bool
        Return values in degrees, otherwise in radians.

    Returns
    ----------
    float list
        Euler angles in [x,y,z] order, radian by default unless specified otherwise.
    """

    # Convert target quaternion to Euler ZYX using scipy package's 'xyz' extrinsic rotation
    # NOTE: scipy uses [x,y,z,w] order to represent quaternion
    eulerZYX = R.from_quat([quat[1], quat[2],
                            quat[3], quat[0]]).as_euler('xyz', degrees=degree).tolist()

    return eulerZYX


def list2str(ls):
    """
    Convert a list to a string.

    Parameters
    ----------
    ls : list
        Source list of any size.

    Returns
    ----------
    str
        A string with format "ls[0] ls[1] ... ls[n] ", i.e. each value 
        followed by a space, including the last one.
    """

    ret_str = ""
    for i in ls:
        ret_str += str(i) + " "
    return ret_str


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
    for pose in root.findall('Pose'):
        position = pose.find('Position')
        orientation = pose.find('Orientation')
        pose_data = [float(position.get('x')), float(position.get('y')), float(position.get('z')), float(orientation.get('qx')), float(orientation.get('qy')), float(orientation.get('qz')), float(orientation.get('qw'))]
        poses.append(pose_data)

    return poses

def pose_to_euler(pose):
    '''
    Convert robot pose from a list [x, y, z, qw, qx, qy, qz] to [x, y, z] and Euler angles.
    
    Parameters:
    pose: list of 7 floats - [x, y, z, qw, qx, qy, qz]
    
    Returns:
    tuple: (x, y, z, roll, pitch, yaw) where (x, y, z) is the position and (roll, pitch, yaw) are the Euler angles in radians.
    '''
    x, y, z, qw, qx, qy, qz = pose
    r = R.from_quat([qx, qy, qz, qw])  # Reordering to match scipy's [qx, qy, qz, qw]
    roll, pitch, yaw = r.as_euler('xyz', degrees=False)
    return [x, y, z, roll, pitch, yaw]

def euler_to_pose(position_euler):
    '''
    Convert robot pose from [x, y, z, roll, pitch, yaw] to [x, y, z, qw, qx, qy, qz].
    
    Parameters:
    position_euler: list of 6 floats - [x, y, z, roll, pitch, yaw]
    
    Returns:
    list: [x, y, z, qw, qx, qy, qz]
    '''
    x, y, z, roll, pitch, yaw = position_euler
    r = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)
    qx, qy, qz, qw = r.as_quat()  # Getting [qx, qy, qz, qw] from scipy
    return [x, y, z, qw, qx, qy, qz]  # Reordering to match [qw, qx, qy, qz]