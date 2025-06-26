import threading
import pyrealsense2 as rs
import cv2
import numpy as np
import re

class RealSenseStreamer:
    def __init__(self, camera_serial):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(camera_serial)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        self.frames = None
        self.running = True
        self.thread = threading.Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()

    def update(self):
        while self.running:
            self.frames = self.pipeline.wait_for_frames()

    def read(self):
        if self.frames:
            depth_frame = self.frames.get_depth_frame()
            color_frame = self.frames.get_color_frame()
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            return color_image, depth_image
        return None, None

    def stop(self):
        self.running = False
        self.thread.join()
        self.pipeline.stop()

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

if __name__ == '__main__':
    cameras = get_attached_rs_devices()
    if not cameras:
        print("No RealSense devices found. Exiting...")
    else:
        streamers = [RealSenseStreamer(camera) for camera in cameras]

        try:
            while True:
                for idx, streamer in enumerate(streamers):
                    color_image, depth_image = streamer.read()
                    if color_image is not None and depth_image is not None:
                        
                        cv2.imshow(f"Camera {idx} - Color", color_image)
                        depth_image_scaled = cv2.convertScaleAbs(depth_image, alpha=0.3)
                        cv2.imshow(f"Camera {idx} - Depth", depth_image_scaled)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            for streamer in streamers:
                streamer.stop()
            cv2.destroyAllWindows()
