import cv2
import numpy as np
import pyrealsense2 as rs
from typing import Type, Tuple, Dict
from dataclasses import dataclass
from PIL import Image
import time

class Camera:
    def __init__(self, device_id: str, width: int = 640, height: int = 480, fps: int = 30):
        self.device_id = device_id
        self.width = width
        self.height = height
        self.fps = fps

        # check RealSense status
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
        time.sleep(2)  # 等待摄像头稳定连接
        
        # Get the stream profile and camera intrinsics
        stream = self.profile.get_stream(rs.stream.color)

        # debug
        intrinsics = stream.as_video_stream_profile().get_intrinsics()
        self.camera_matrix_test = np.array([[intrinsics.fx, 0, intrinsics.ppx], 
                                      [0, intrinsics.fy, intrinsics.ppy], 
                                      [0, 0, 1]], dtype=float)
        print(f'self.camera_matrix_test:{self.camera_matrix_test}')

        #TODO: camera matrix is wrong
        self.camera_matrix = np.array([[607.168, 0, 325.575], [0, 606.94, 325.575], [0, 0, 1]], dtype=float)
        self.dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion

        # Load the predefined dictionary
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()

    # check realsense status
    def check_realsense_connection(self) -> bool:
        context = rs.context()
        if len(context.devices) == 0:
            print("No RealSense devices connected.")
            return False
        else:
            print("RealSense device detected.")
            return True
            
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


    def display(self, option="rgb"):
        try:
            while True:
                try:
                    # 设置帧获取超时时间为5000毫秒（5秒）
                    frames = self.pipeline.wait_for_frames(5000)

                    # 根据 option 选择性获取帧
                    color_frame = frames.get_color_frame() if option in ["rgb", "rgbd"] else None
                    depth_frame = frames.get_depth_frame() if option in ["d", "rgbd"] else None

                    # 检查帧是否成功获取
                    if option in ["rgb", "rgbd"] and not color_frame:
                        print("RGB frame not received. Retrying...")
                        continue
                    if option in ["d", "rgbd"] and not depth_frame:
                        print("Depth frame not received. Retrying...")
                        continue

                    # 显示 RGB 图像
                    if color_frame:
                        color_image = np.asanyarray(color_frame.get_data())
                        cv2.imshow('RGB Frame', color_image)

                    # 显示深度图像
                    if depth_frame:
                        depth_image = np.asanyarray(depth_frame.get_data())
                        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                        cv2.imshow('Depth Frame', depth_colormap)

                    # Press 'q' to exit
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

                except RuntimeError as e:
                    print(f"Error receiving frame: {e}")
                    continue
        finally:
            # Stop streaming and close windows
            self.pipeline.stop()
            cv2.destroyAllWindows()