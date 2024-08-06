import cv2
import pyrealsense2 as rs
import numpy as np

def transform_to_robot_base(marker_positions, tcp_pose, camera_to_tcp_transform):
    # Convert tcp_pose to a transformation matrix
    tcp_matrix = pose_to_transformation_matrix(tcp_pose)
    
    # Convert camera_to_tcp_transform to a transformation matrix
    camera_matrix = pose_to_transformation_matrix(camera_to_tcp_transform)
    
    # Combine transformations
    total_transform = np.dot(tcp_matrix, camera_matrix)
    
    # Transform marker positions to robot base frame
    marker_positions_homogeneous = np.hstack((marker_positions, np.ones((marker_positions.shape[0], 1))))
    transformed_positions = np.dot(total_transform, marker_positions_homogeneous.T).T
    
    return transformed_positions[:, :3]

def pose_to_transformation_matrix(pose):
    # Convert pose to transformation matrix
    translation = pose[:3]
    rotation = pose[3:]
    rotation_matrix, _ = cv2.Rodrigues(rotation)
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix
    transformation_matrix[:3, 3] = translation
    return transformation_matrix

def get_marker_positions(debug=False):
    # Configure RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device('239722070506')
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    
    # Start streaming
    profile = pipeline.start(config)

    # Get the stream profile and camera intrinsics
    stream = profile.get_stream(rs.stream.color)  # Fetch stream profile for color stream
    intrinsics = stream.as_video_stream_profile().get_intrinsics()
    print("Camera intrinsic: ",intrinsics)

    # Load the predefined dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters()  # Initialize detector parameters

    # Camera matrix and distortion coefficients (dummy values, replace with real calibration data)

    # intrinsic = np.array([[496.1664368,    0.       ,  317.43565878],
    #                             [  0.  ,       495.2396371 , 242.50319213],
    #                             [  0.  ,         0.         ,  1.        ]])


    # dist = np.array([[ 2.34901945e-01, -1.71615876e+00, 3.07223172e-03, -1.40436182e-03, 3.96554659e+00]])


    camera_matrix = np.array([[607.168, 0, 325.575], [0, 606.94, 325.575], [0, 0, 1]], dtype=float)
    dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion

    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            
            if not color_frame:
                continue

            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())

            # Convert to grayscale
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

            # Detect ArUco markers in the image
            corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            marker_positions = {}

            # If markers are detected
            if ids is not None:
                # Draw the detected markers
                cv2.aruco.drawDetectedMarkers(color_image, corners, ids)

                for i in range(len(ids)):
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.035, camera_matrix, dist_coeffs)
                    # Draw the axis for each marker
                    if hasattr(cv2.aruco, 'drawAxis'):
                        cv2.aruco.drawAxis(color_image, camera_matrix, dist_coeffs, rvec, tvec, 0.1)
                    # Store the position in a dictionary
                    marker_positions[ids[i][0]] = tvec[0][0]

                # Convert the positions dictionary to a NumPy array
                # positions_array = np.array([marker_positions[key] for key in marker_positions])
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
        pipeline.stop()
        if debug:
            cv2.destroyAllWindows()

# Example usage
# get marker
if __name__ == "__main__":
    marker_positions = get_marker_positions(debug=True)
    if marker_positions is not None:
        print("Marker positions in camera frame:\n", marker_positions)
        
        # Example TCP pose and camera-to-TCP transform
        tcp_pose = np.array([0.560, -0.081, 0.479])  # Replace with actual TCP pose
        # camera_to_tcp_transform = np.array([-0.074719, 0, 0.148997 , 0, 0, 0])  # Replace with actual camera-to-TCP transform
        
        # Transform marker positions to robot base frame
        pos_cam_world = tcp_pose + np.array([0.074719,
                                            0,
                                            -0.148997-0.06])

        pos_marker_world = pos_cam_world + marker_positions
        print("Marker positions in robot base frame:\n", pos_marker_world)
    else:
        print("No markers detected.")