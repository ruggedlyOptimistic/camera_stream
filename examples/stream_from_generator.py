import pyrealsense2 as rs
import cv2
import numpy as np
import re

def get_attached_rs_devices():
    devices_from_ctx = rs.context().query_devices()
    serial_pattern = r'N: (\d{12})'
    usb_pattern = r'USB.{3}'
    devices_by_serial = []

    for device in devices_from_ctx:
        match = re.search(serial_pattern, str(device))
        usb_type = re.search(usb_pattern, str(device)).group()

        if match:
            if usb_type in ["USB3.2", "USB3.1", "USB3.0"]:
                devices_by_serial.append(match.group(1))
            else:
                print(f"RealSense Device {match.group(1)} found with unsupported USB connection type! Skipping this device...")

    if len(devices_by_serial) == 0:
        print("No Real Sense Devices Found!")

    return devices_by_serial

def Generator_FromStream(cameras, iStart=0, iEnd=1, DirRunData='run_data'):
    pipelines = [None] * len(cameras)  # setup a fixed-length list of pipelines for each camera stream
    configs = [None] * len(cameras)

    for i in range(len(cameras)):
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(cameras[i])
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

        pipelines[i] = pipeline
        configs[i] = config

    for i in range(len(cameras)):
        print(f'Starting pipeline for camera {cameras[i]}...', end="")
        pipelines[i].start(configs[i])
        print('done')

    try:
        while True:
            for idx, pipeline in enumerate(pipelines):
                frames = pipeline.poll_for_frames()
                if not frames:
                    continue

                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                if not depth_frame or not color_frame:
                    continue

                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                yield {
                    'ImageDepth': depth_image,
                    'ImageColor': color_image,
                    'CameraIndex': idx
                }
    finally:
        for active_stream in pipelines:
            active_stream.stop()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    cameras = get_attached_rs_devices()
    if not cameras:
        print("No RealSense devices found. Exiting...")
    else:
        for image_data in Generator_FromStream(cameras):
            color_image = image_data['ImageColor']
            depth_image = image_data['ImageDepth']
            idx = image_data['CameraIndex']

            cv2.imshow(f"Camera {idx} - Color", color_image)
            depth_image_scaled = cv2.convertScaleAbs(depth_image, alpha=0.3)  # Fixed typo 'converScaleAbs'
            cv2.imshow(f"Camera {idx} - Depth", depth_image_scaled)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()
