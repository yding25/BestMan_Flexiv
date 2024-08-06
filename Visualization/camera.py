import cv2
import numpy as np
import pyrealsense2 as rs
from typing import Type, Tuple, Dict
from dataclasses import dataclass
from PIL import Image

class Camera:
    def __init__(self, device_id: str, width: int = 640, height: int = 480, fps: int = 30):
        self.device_id = device_id
        self.width = width
        self.height = height
        self.fps = fps

        # Configure RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_device(self.device_id)
        self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)

        # Start streaming
        self.profile = self.pipeline.start(self.config)
        
        # Get the stream profile and camera intrinsics
        stream = self.profile.get_stream(rs.stream.color)
        intrinsics = stream.as_video_stream_profile().get_intrinsics()
        self.camera_matrix = np.array([[intrinsics.fx, 0, intrinsics.ppx], 
                                       [0, intrinsics.fy, intrinsics.ppy], 
                                       [0, 0, 1]], dtype=float)
        self.dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion

        # Load the predefined dictionary
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()

    def update(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None
        
        color_image = np.asanyarray(color_frame.get_data())
        return color_image

    def get_marker_positions(self, debug: bool = False) -> np.ndarray:
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
