import pyrealsense2 as rs

def list_realsense_devices():
    # Create a context object to manage RealSense devices
    context = rs.context()

    # Check if there are any connected devices
    if len(context.devices) == 0:
        print("No RealSense devices found.")
        return None

    # List all connected devices
    for i, device in enumerate(context.devices):
        print(f"Device {i + 1}: {device.get_info(rs.camera_info.name)}")
        print(f"  Serial Number (device_id): {device.get_info(rs.camera_info.serial_number)}")
        print(f"  Firmware Version: {device.get_info(rs.camera_info.firmware_version)}")

# Call the function to list devices
list_realsense_devices()
