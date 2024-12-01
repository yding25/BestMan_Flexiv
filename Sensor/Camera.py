# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : Camera.py
# @Time           : 2024-12-02 01:45:01
# @Author         : Yan
# @Email          : yding25@binghamton.edu
# @Description    : Camera functions
# @Usage          : None
"""


import cv2
import numpy as np
import pyrealsense2 as rs
from typing import Dict
import time
import threading
import signal

class Camera:
    def __init__(self, device_id: str, width: int = 640, height: int = 480, fps: int = 30):
        """
        Initializes the Camera class with RealSense device parameters.

        Args:
            device_id (str): The ID of the RealSense device.
            width (int): The width of the image stream (default is 640).
            height (int): The height of the image stream (default is 480).
            fps (int): Frames per second for the image stream (default is 30).
        """
        self.device_id = device_id
        self.width = width
        self.height = height
        self.fps = fps

        # Check if a RealSense device is connected
        if not self.check_realsense_connection():
            raise RuntimeError("No RealSense device connected.")
        
        # Configure RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_device(self.device_id)
        self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # Start streaming
        self.profile = self.pipeline.start(self.config)
        time.sleep(2)  # Wait for the camera to stabilize
        
        # Get the stream profile and camera intrinsics
        stream = self.profile.get_stream(rs.stream.color)

        # Debug: Extract camera intrinsic parameters
        intrinsics = stream.as_video_stream_profile().get_intrinsics()
        self.camera_matrix_test = np.array([[intrinsics.fx, 0, intrinsics.ppx], 
                                      [0, intrinsics.fy, intrinsics.ppy], 
                                      [0, 0, 1]], dtype=float)
        print(f'self.camera_matrix_test:{self.camera_matrix_test}')

        #TODO: camera matrix is wrong
        self.camera_matrix = np.array([[607.168, 0, 325.575], [0, 606.94, 325.575], [0, 0, 1]], dtype=float)
        self.dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion

        # Load the predefined ArUco marker dictionary
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()

    # check realsense status
    def check_realsense_connection(self) -> bool:
        """
        Checks if any RealSense device is connected.

        Returns:
            bool: True if a device is detected, False otherwise.
        """
        context = rs.context()
        if len(context.devices) == 0:
            print("No RealSense devices connected.")
            return False
        else:
            print("RealSense device detected.")
            return True
            
    def update(self):
        """
        Captures the latest color frame from the camera.

        Returns:
            np.ndarray: The color frame as a NumPy array, or None if not available.
        """
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None
        
        color_image = np.asanyarray(color_frame.get_data())
        return color_image

    def get_marker_positions(self, debug: bool = False) -> np.ndarray:
        """
        Detects ArUco markers in the camera's field of view and returns their positions.

        Args:
            debug (bool): If True, displays the frame with markers for debugging.

        Returns:
            np.ndarray: An array of marker positions in the format [y, x, -z].
        """
        try:
            while True:
                color_image = self.update()
                if color_image is None:
                    continue
                
                # Convert to grayscale
                gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
                
                # Detect ArUco markers in the image
                corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
                
                marker_positions: Dict[int, np.ndarray] = {}
                
                # If markers are detected
                if ids is not None:
                    # Draw the detected markers
                    cv2.aruco.drawDetectedMarkers(color_image, corners, ids)
                    
                    for i in range(len(ids)):
                        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.035, self.camera_matrix, self.dist_coeffs)
                        # Draw the axis for each marker
                        if hasattr(cv2.aruco, 'drawAxis'):
                            cv2.aruco.drawAxis(color_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
                        # Store the position in a dictionary
                        marker_positions[ids[i][0]] = tvec[0][0]

                    # Convert the positions dictionary to a NumPy array
                    positions_array = np.array([[pos[1], pos[0], -pos[2]] for pos in marker_positions.values()])
                    
                    # Debug mode: Show the frame with markers
                    if debug:
                        cv2.imshow('RealSense Debug', color_image)
                        if cv2.waitKey(1000) & 0xFF == ord('q'):
                            break
                    
                    return positions_array

                # In debug mode, show the frame even if no markers are detected
                if debug:
                    cv2.imshow('RealSense Debug', color_image)
                    if cv2.waitKey(1000) & 0xFF == ord('q'):
                        break

            return None
        finally:
            # Stop streaming
            self.pipeline.stop()
            if debug:
                cv2.destroyAllWindows()


    def display(self, option="rgb"):
        """
        Displays frames from the RealSense camera in a separate thread. 
        Supports RGB, depth, or both.

        Parameters:
            option (str): Determines the display mode.
                        "rgb" - display only RGB frames.
                        "d" - display only depth frames.
                        "rgbd" - display both RGB and depth frames.

        This function listens for the Ctrl+C signal to stop gracefully.
        """
        stop_flag = [False]  # Mutable flag to control the thread termination

        def signal_handler(sig, frame):
            """
            Signal handler for gracefully exiting the program on Ctrl+C.

            Args:
                sig: Signal number.
                frame: Current stack frame.
            """
            stop_flag[0] = True
            print("Ctrl+C detected. Exiting...")

        # Bind the SIGINT signal (Ctrl+C) to the signal handler
        signal.signal(signal.SIGINT, signal_handler)

        def process_frames():
            """
            Fetches frames from the RealSense camera and displays them 
            in a loop until the stop_flag is set to True.
            """
            try:
                while not stop_flag[0]:
                    # Fetch frames non-blocking
                    frames = self.pipeline.poll_for_frames()
                    if not frames:
                        continue

                    # Retrieve color and depth frames based on the selected option
                    color_frame = frames.get_color_frame() if option in ["rgb", "rgbd"] else None
                    depth_frame = frames.get_depth_frame() if option in ["d", "rgbd"] else None

                    # Skip iteration if frames are not available for the selected mode
                    if option in ["rgb", "rgbd"] and not color_frame:
                        continue
                    if option in ["d", "rgbd"] and not depth_frame:
                        continue

                    # Display RGB image if available
                    if color_frame:
                        color_image = np.asanyarray(color_frame.get_data())
                        cv2.imshow('RGB Frame', color_image)

                    # Display depth image if available
                    if depth_frame:
                        depth_image = np.asanyarray(depth_frame.get_data())
                        # Normalize depth data and apply color mapping for visualization
                        depth_colormap = cv2.convertScaleAbs(depth_image, alpha=255.0 / depth_image.max())
                        cv2.imshow('Depth Frame', depth_colormap)

                    # Exit loop if 'q' key is pressed
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
            finally:
                # Ensure resources are properly released
                self.pipeline.stop()  # Stop the RealSense pipeline
                cv2.destroyAllWindows()  # Close OpenCV display windows

        # Start the frame processing in a separate thread
        thread = threading.Thread(target=process_frames)
        thread.start()

        # Wait for the thread to finish (main thread blocks here)
        thread.join()
