"""Driver class for SpaceMouse controller. Modified based on the robosuite code.

This class provides a driver support to SpaceMouse on Mac OS X.
In particular, we assume you are using a SpaceMouse Wireless by default.

To set up a new SpaceMouse controller:
    1. Download and install driver from https://www.3dconnexion.com/service/drivers.html
    2. Install hidapi library through pip
       (make sure you run uninstall hid first if it is installed).
    3. Make sure SpaceMouse is connected before running the script
    4. (Optional) Based on the model of SpaceMouse, you might need to change the
       vendor id and product id that correspond to the device.

For Linux support, you can find open-source Linux drivers and SDKs online.
    See http://spacenav.sourceforge.net/

"""
import pyspacemouse
import time
import threading
import time
from collections import namedtuple
import numpy as np
from Bestman_flexiv import *
import math
from scipy.spatial.transform import Rotation as R
try:
    import hid
    # for device_info in hid.enumerate():
    # # 打印每个设备的信息
    #     print("="*40)
    #     print(f"Vendor ID: {device_info['vendor_id']:04x}")
    #     print(f"Product ID: {device_info['product_id']:04x}")
    #     print(f"Product: {device_info['product_string']}")
    #     print(f"Manufacturer: {device_info['manufacturer_string']}")
    #     print(f"Path: {device_info['path']}")
    #     print("="*40)
except ModuleNotFoundError as exc:
    raise ImportError(
        "Unable to load module hid, required to interface with SpaceMouse. "
        "Only Mac OS X is officially supported. Install the additional "
        "requirements with `pip install -r requirements-ik.txt`"
    ) from exc

# from deoxys.utils.transform_utils import rotation_matrix

def button_0(state, buttons, pressed_buttons):
    print("Button:", pressed_buttons)


def button_0_1(state, buttons, pressed_buttons):
    print("Buttons:", pressed_buttons)


def someButton(state, buttons):
    print("Some button")


def rotation_matrix(angle, direction, point=None):
    """
    Returns matrix to rotate about axis defined by point and direction.

    E.g.:
        >>> angle = (random.random() - 0.5) * (2*math.pi)
        >>> direc = numpy.random.random(3) - 0.5
        >>> point = numpy.random.random(3) - 0.5
        >>> R0 = rotation_matrix(angle, direc, point)
        >>> R1 = rotation_matrix(angle-2*math.pi, direc, point)
        >>> is_same_transform(R0, R1)
        True

        >>> R0 = rotation_matrix(angle, direc, point)
        >>> R1 = rotation_matrix(-angle, -direc, point)
        >>> is_same_transform(R0, R1)
        True

        >>> I = numpy.identity(4, numpy.float32)
        >>> numpy.allclose(I, rotation_matrix(math.pi*2, direc))
        True

        >>> numpy.allclose(2., numpy.trace(rotation_matrix(math.pi/2,
        ...                                                direc, point)))
        True

    Args:
        angle (float): Magnitude of rotation
        direction (np.array): (ax,ay,az) axis about which to rotate
        point (None or np.array): If specified, is the (x,y,z) point about which the rotation will occur

    Returns:
        np.array: 4x4 homogeneous matrix that includes the desired rotation
    """
    sina = math.sin(angle)
    cosa = math.cos(angle)
    direction = unit_vector(direction[:3])
    # rotation matrix around unit vector
    R = np.array(
        ((cosa, 0.0, 0.0), (0.0, cosa, 0.0), (0.0, 0.0, cosa)), dtype=np.float32
    )
    R += np.outer(direction, direction) * (1.0 - cosa)
    direction *= sina
    R += np.array(
        (
            (0.0, -direction[2], direction[1]),
            (direction[2], 0.0, -direction[0]),
            (-direction[1], direction[0], 0.0),
        ),
        dtype=np.float32,
    )
    M = np.identity(4)
    M[:3, :3] = R
    if point is not None:
        # rotation not around origin
        point = np.array(point[:3], dtype=np.float32, copy=False)
        M[:3, 3] = point - np.dot(R, point)
    return M


def unit_vector(data, axis=None, out=None):
    """
    Returns ndarray normalized by length, i.e. eucledian norm, along axis.

    E.g.:
        >>> v0 = numpy.random.random(3)
        >>> v1 = unit_vector(v0)
        >>> numpy.allclose(v1, v0 / numpy.linalg.norm(v0))
        True

        >>> v0 = numpy.random.rand(5, 4, 3)
        >>> v1 = unit_vector(v0, axis=-1)
        >>> v2 = v0 / numpy.expand_dims(numpy.sqrt(numpy.sum(v0*v0, axis=2)), 2)
        >>> numpy.allclose(v1, v2)
        True

        >>> v1 = unit_vector(v0, axis=1)
        >>> v2 = v0 / numpy.expand_dims(numpy.sqrt(numpy.sum(v0*v0, axis=1)), 1)
        >>> numpy.allclose(v1, v2)
        True

        >>> v1 = numpy.empty((5, 4, 3), dtype=numpy.float32)
        >>> unit_vector(v0, axis=1, out=v1)
        >>> numpy.allclose(v1, v2)
        True

        >>> list(unit_vector([]))
        []

        >>> list(unit_vector([1.0]))
        [1.0]

    Args:
        data (np.array): data to normalize
        axis (None or int): If specified, determines specific axis along data to normalize
        out (None or np.array): If specified, will store computation in this variable

    Returns:
        None or np.array: If @out is not specified, will return normalized vector. Otherwise, stores the output in @out
    """
    if out is None:
        data = np.array(data, dtype=np.float32, copy=True)
        if data.ndim == 1:
            data /= math.sqrt(np.dot(data, data))
            return data
    else:
        if out is not data:
            out[:] = np.array(data, copy=False)
        data = out
    length = np.atleast_1d(np.sum(data * data, axis))
    np.sqrt(length, length)
    if axis is not None:
        length = np.expand_dims(length, axis)
    data /= length
    if out is None:
        return data


AxisSpec = namedtuple("AxisSpec", ["channel", "byte1", "byte2", "scale"])

SPACE_MOUSE_SPEC = {
    "x": AxisSpec(channel=1, byte1=1, byte2=2, scale=1),
    "y": AxisSpec(channel=1, byte1=3, byte2=4, scale=-1),
    "z": AxisSpec(channel=1, byte1=5, byte2=6, scale=-1),
    "roll": AxisSpec(channel=1, byte1=7, byte2=8, scale=-1),
    "pitch": AxisSpec(channel=1, byte1=9, byte2=10, scale=-1),
    "yaw": AxisSpec(channel=1, byte1=11, byte2=12, scale=1),
}


def to_int16(y1, y2):
    """
    Convert two 8 bit bytes to a signed 16 bit integer.

    Args:
        y1 (int): 8-bit byte
        y2 (int): 8-bit byte

    Returns:
        int: 16-bit integer
    """
    x = (y1) | (y2 << 8)
    if x >= 32768:
        x = -(65536 - x)
    return x


def scale_to_control(x, axis_scale=350.0, min_v=-1.0, max_v=1.0):
    """
    Normalize raw HID readings to target range.

    Args:
        x (int): Raw reading from HID
        axis_scale (float): (Inverted) scaling factor for mapping raw input value
        min_v (float): Minimum limit after scaling
        max_v (float): Maximum limit after scaling

    Returns:
        float: Clipped, scaled input from HID
    """
    x = x / axis_scale
    x = min(max(x, min_v), max_v)
    return x


def convert(b1, b2):
    """
    Converts SpaceMouse message to commands.

    Args:
        b1 (int): 8-bit byte
        b2 (int): 8-bit byte

    Returns:
        float: Scaled value from Spacemouse message
    """
    return scale_to_control(to_int16(b1, b2))


class SpaceMouse:
    """
    A minimalistic driver class for SpaceMouse with HID library.

    Note: Use hid.enumerate() to view all USB human interface devices (HID).
    Make sure SpaceMouse is detected before running the script.
    You can look up its vendor/product id from this method.

    Args:
        vendor_id (int): HID device vendor id
        product_id (int): HID device product id
        pos_sensitivity (float): Magnitude of input position command scaling
        rot_sensitivity (float): Magnitude of scale input rotation commands scaling
    """

    def __init__(
        self, vendor_id=9583, product_id=50734, pos_sensitivity=1.0, rot_sensitivity=1.0
    ):

        print("Opening SpaceMouse device")
        self.product_id = product_id
        self.device = hid.device()
        self.device.open(vendor_id, product_id)  # SpaceMouse

        self.pos_sensitivity = pos_sensitivity
        self.rot_sensitivity = rot_sensitivity

        print("Manufacturer: %s" % self.device.get_manufacturer_string())
        print("Product: %s" % self.device.get_product_string())

        # 6-DOF variables
        self.x, self.y, self.z = 0, 0, 0
        self.roll, self.pitch, self.yaw = 0, 0, 0
        self._display_controls()

        self.single_click_and_hold = False

        self._control = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._reset_state = 0
        self.rotation = np.array([[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]])
        self._enabled = False

        # launch a new listener thread to listen to SpaceMouse
      
        self.thread = threading.Thread(target=self.run)
        self.thread.daemon = True
        self.thread.start()

            

    @staticmethod
    def _display_controls():
        """
        Method to pretty print controls.
        """

        def print_command(char, info):
            char += " " * (30 - len(char))
            print("{}\t{}".format(char, info))

        print("")
        print_command("Control", "Command")
        print_command("Right button", "reset simulation")
        print_command("Left button (hold)", "close gripper")
        print_command("Move mouse laterally", "move arm horizontally in x-y plane")
        print_command("Move mouse vertically", "move arm vertically")
        print_command(
            "Twist mouse about an axis", "rotate arm about a corresponding axis"
        )
        print_command("ESC", "quit")
        print("")

    def _reset_internal_state(self):
        """
        Resets internal state of controller, except for the reset signal.
        """
        self.rotation = np.array([[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]])
        # Reset 6-DOF variables
        self.x, self.y, self.z = 0, 0, 0
        self.roll, self.pitch, self.yaw = 0, 0, 0
        # Reset control
        self._control = np.zeros(6)
        # Reset grasp
        self.single_click_and_hold = False

    def button_down(self):
        self.single_click_and_hold = True



    def start_control(self):
        """
        Method that should be called externally before controller can
        start receiving commands.
        """
        self._reset_internal_state()
        self._reset_state = 0
        self._enabled = True

    def get_controller_state(self):
        """
        Grabs the current state of the 3D mouse.

        Returns:
            dict: A dictionary containing dpos, orn, unmodified orn, grasp, and reset
        """
        dpos = self.control[:3] * 0.005 * self.pos_sensitivity
        roll, pitch, yaw = self.control[3:] * 0.005 * self.rot_sensitivity

        # convert RPY to an absolute orientation
        drot1 = rotation_matrix(angle=-pitch, direction=[1.0, 0, 0], point=None)[:3, :3]
        drot2 = rotation_matrix(angle=roll, direction=[0, 1.0, 0], point=None)[:3, :3]
        drot3 = rotation_matrix(angle=yaw, direction=[0, 0, 1.0], point=None)[:3, :3]

        self.rotation = self.rotation.dot(drot1.dot(drot2.dot(drot3)))

        return dict(
            dpos=dpos,
            rotation=self.rotation,
            raw_drotation=np.array([roll, pitch, yaw]),
            grasp=self.control_gripper,
            reset=self._reset_state,
        )

    def run(self):
        """Listener method that keeps pulling new messages."""

        t_last_click = -1
        # while True:
        #     d = self.device.read(13)
        #     print(d)

        #for wireless spacemouse:
        if self.product_id == 50734:
            while True:
                d = self.device.read(13)
                if d is not None and self._enabled:
                    print(d)
                    if d[0] == 1 and self.product_id == 50734:  ## readings from 6-DoF sensor
                        self.y = convert(d[1], d[2])
                        self.x = convert(d[3], d[4])
                        self.z = convert(d[5], d[6]) * -1.0

                        self.roll = convert(d[7], d[8])
                        self.pitch = convert(d[9], d[10])
                        self.yaw = convert(d[11], d[12])

                        self._control = [
                            self.x,
                            self.y,
                            self.z,
                            self.roll,
                            self.pitch,
                            self.yaw,
                        ]
                        # print(self._control)
                    elif d[0]  == 1:
                        self.x = scale_to_control(d[1])
                        self.y = scale_to_control(d[2])
                        self.z = scale_to_control(d[3])

                        self.roll = scale_to_control(d[4])
                        self.pitch = scale_to_control(d[5])
                        self.yaw = scale_to_control(d[6])

                        self._control = [
                            self.x,
                            self.y,
                            self.z,
                            self.roll,
                            self.pitch,
                            self.yaw,
                        ]
                        print(self._control)
                    elif d[0] == 3:  ## readings from the side buttons

                        # press left button
                        if d[1] == 1:
                            t_click = time.time()
                            elapsed_time = t_click - t_last_click
                            t_last_click = t_click
                            self.single_click_and_hold = True

                        # release left button
                        if d[1] == 0:
                            self.single_click_and_hold = False

                        # right button is for reset
                        if d[1] == 2:
                            self._reset_state = 1
                            self._enabled = False
                            self._reset_internal_state()
                            
        elif self.product_id == 50741:
            success = pyspacemouse.open()
            if success:
                while True:
                    state = pyspacemouse.read()
                    self._control = [
                            -state.y,
                            state.x,
                            state.z,
                            -state.roll,
                            -state.pitch,
                            -state.yaw,
                        ]
                    button = state.buttons
                    if button[0] == 1:
                        self.single_click_and_hold = True
                    else:
                        self.single_click_and_hold = False
                    # print(state.x)
                    time.sleep(0.01)
    @property
    def control(self):
        """
        Grabs current pose of Spacemouse

        Returns:
            np.array: 6-DoF control value
        """
        return np.array(self._control)

    @property
    def control_gripper(self):
        """
        Maps internal states into gripper commands.

        Returns:
            float: Whether we're using single click and hold or not
        """
        if self.single_click_and_hold:
            return 1.0
        return 0



def input2action(device, controller_type="OSC_POSE", robot_name="Panda", gripper_dof=1):
    state = device.get_controller_state()
    # Note: Devices output rotation with x and z flipped to account for robots starting with gripper facing down
    #       Also note that the outputted rotation is an absolute rotation, while outputted dpos is delta pos
    #       Raw delta rotations from neutral user input is captured in raw_drotation (roll, pitch, yaw)
    dpos, rotation, raw_drotation, grasp, reset = (
        state["dpos"],
        state["rotation"],
        state["raw_drotation"],
        state["grasp"],
        state["reset"],
    )

    drotation = raw_drotation[[1, 0, 2]]

    action = None

    if not reset:
        if controller_type == "OSC_POSE":
            drotation[2] = -drotation[2]
            drotation *= 75
            dpos *= 200
            drotation = drotation

            grasp = 1 if grasp else -1
            action = np.concatenate([dpos, drotation, [grasp] * gripper_dof])

        if controller_type == "OSC_YAW":
            drotation[2] = -drotation[2]
            drotation *= 75
            dpos *= 200

            grasp = 1 if grasp else -1
            action = np.concatenate([dpos, drotation, [grasp] * gripper_dof])

            # drotation = T.quat2axisangle(T.mat2quat(T.euler2mat(drotation)))
        if controller_type == "OSC_POSITION":
            drotation[:] = 0
            dpos *= 200
            grasp = 1 if grasp else -1
            action = np.concatenate([dpos, drotation, [grasp] * gripper_dof])

            # drotation = T.quat2axisangle(T.mat2quat(T.euler2mat(drotation)))

        if controller_type == "JOINT_IMPEDANCE":
            grasp = 1 if grasp else -1
            action = np.array([0.0] * 7 + [grasp] * gripper_dof)

    return action, grasp



def move_with_spacemouse(bestman, action_num=1000, vendor_id=9583, product_id=50741):
    device = SpaceMouse(vendor_id=vendor_id, product_id=product_id)
    device.start_control()

    log = flexivrdk.Log()
    # controller_cfg = YamlConfig("config/osc-pose-controller.yml").as_easydict()

    # robot_interface._state_buffer = []
    try:
        time.sleep(1)
        last_gripper_state = 0
        #starting control
        #for wireless spacemouse:
        if product_id == 50734:

            log = flexivrdk.Log()
            for i in range(action_num):
                # start_time = time.time_ns()
                action, grasp = input2action(
                    device=device
                )
                if i == action_num-1:
                    action[0:6] = [0.0] * 6
                current_gripper_state = action[6]
                if current_gripper_state != last_gripper_state:
                    if current_gripper_state == 1:
                        bestman.close_gripper()
                    else:
                        bestman.open_gripper()
                    last_gripper_state = current_gripper_state
                current_pos = bestman.get_current_end_effector_pose()
                current_euler = current_pos[3:]

                action_euler = action[3:6]
                R_current = R.from_euler('xyz', current_euler)
                R_action = R.from_euler('xyz', action_euler)
                R_target = R_action * R_current
                euler_angles_final = R_target.as_euler('xyz')

                target_pos = list(current_pos[0:3] + action[0:3]) + list(euler_angles_final)
                bestman.move_end_effector_to_goal_pose(target_pos, max_linear_vel=0.1, max_angular_vel=0.5)
                time.sleep(0.05)
        elif product_id == 50741:
            for i in range(action_num):
                state = pyspacemouse.read()
                controller = [
                            -state.y / 3.0,
                            state.x / 3.0,
                            state.z / 3.0,
                            -state.roll / 3.0,
                            -state.pitch / 3.0,
                            -state.yaw / 3.0,
                        ]
                if i == action_num - 1:
                    controller = [0.0] * 6
                current_gripper_state = state.buttons[0]
                if current_gripper_state != last_gripper_state:
                    if current_gripper_state == 1:
                        bestman.close_gripper()
                    else:
                        bestman.open_gripper()
                    last_gripper_state = current_gripper_state
                current_pos = bestman.get_current_end_effector_pose()
                current_euler = current_pos[3:]
                action_euler = controller[3:6]
                R_current = R.from_euler('xyz', current_euler)
                R_action = R.from_euler('xyz', action_euler)
                R_target = R_action * R_current
                euler_angles_final = R_target.as_euler('xyz')

                target_pos = list(np.array(current_pos[0:3]) + np.array(controller[0:3])) + list(euler_angles_final)
                bestman.move_end_effector_to_goal_pose(target_pos, max_linear_vel=0.1, max_angular_vel=0.5)
                time.sleep(0.02)

    except Exception as e:
        # Log any exceptions that occur
        log.error(str(e))


if __name__ == "__main__":

    space_mouse = SpaceMouse(product_id=50741)
    for i in range(100):
        print(space_mouse.control, space_mouse.control_gripper)
        time.sleep(0.02)
