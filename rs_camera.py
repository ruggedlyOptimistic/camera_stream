import pyrealsense2 as rs
import threading
import numpy as np

import re, cv2

class RS_Camera:

    def __init__(self, system_params, serial, cxn):
        camera_params = system_params['serial'][serial]
        self.cxn = cxn  # Connection type
        self.serial = serial  # Device Serial # from context

        self.resolution_width = system_params['resolution']['width']
        self.resolution_height = system_params['resolution']['height']
        self.frame_rate = system_params['frame_rate']
        self.depth_aligned = system_params['depth_aligned']

        self.model = camera_params['model']
        self.location = camera_params['location']
        self.depth_enabled = camera_params['depth']['enabled']
        self.color_enabled = camera_params['color']['enabled']

        depth_format = rs.format.z16  # Standard format for depth images
        color_format = rs.format.bgr8  # Standard format for color images

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        self.config.enable_device(self.serial)
        self.config.enable_stream(rs.stream.depth, self.resolution_width, self.resolution_height, depth_format, self.frame_rate)
        self.config.enable_stream(rs.stream.color, self.resolution_width, self.resolution_height, color_format, self.frame_rate)

        self.frames = None
        self.aligned_frames = None
        self.running = False
        self.thread = None

        if self.is_valid_cxn_type():
            self.align_to = rs.stream.color
            self.align = rs.align(self.align_to)
        else:
            print(f"Invalid connection type for camera with serial {self.serial}")

    def get_model(self):
        return self.model
    
    def get_serial(self):
        return self.serial

    def get_status(self):
        return self.running
    
    def get_robot_location(self):
        return self.location

    def update(self):
        while self.running:
            frames = self.pipeline.wait_for_frames()
            self.aligned_frames = self.align.process(frames) if self.depth_aligned else frames

    def read(self):
        color_image, depth_image = None, None

        if self.aligned_frames:
            aligned_depth_frame = self.aligned_frames.get_depth_frame()
            color_frame = self.aligned_frames.get_color_frame()

            if aligned_depth_frame and color_frame:
                depth_image = np.asanyarray(aligned_depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())    
                
            else:
                print(f"Error reading from camera at robot location {self.location}")

        return color_image, depth_image

    def start(self):        
        self.pipeline.start(self.config)
        self.running = True
        self.thread = threading.Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()

    def stop(self):
        if self.running:
            self.running = False
            if self.thread is not None:
                self.thread.join()
            self.pipeline.stop()

    def set_roi(self, r1, r2, c1, c2):
        try:
            self.ROI = np.array([[r1, r2], [c1, c2]]).astype(int)

            if r1 < 0 or c1 < 0 or r2 > self.resolution_height or c2 > self.resolution_width:
                raise IndexError("One or more ROI boundaries are outside viewport resolution!")
        except TypeError:
            print(f"TypeError encountered! Resetting ROI to {self.resolution_width} x {self.resolution_height}")
            self.ROI = np.array([[0, self.resolution_height], [0, self.resolution_width]])

        except IndexError:
            print(f"Resetting ROI to {self.resolution_width} x {self.resolution_height}")
            self.ROI = np.array([[0, self.resolution_height], [0, self.resolution_width]])

    def is_valid_cxn_type(self):
        # Placeholder for actual logic to validate connection type
        return self.cxn in ["USB3.2", "USB3.1", "USB3.0"]

#########  Debugging Only  #############################

def load_camera_params():
    params = {
        'resolution': {'width': 640, 'height': 480},
        'frame_rate': 30,
        'depth_aligned': True,
        'serial': {
            '311322301090': {
                'model': 'D456',
                'location': 'A',
                'depth': {'enabled': 1, 'format': 'z16'},
                'color': {'enabled': 1, 'format': 'bgr8'},
            },
            '309622300248': {
                'model': 'D456',
                'location': 'D',
                'depth': {'enabled': 1, 'format': 'z16'},
                'color': {'enabled': 1, 'format': 'bgr8'},
            }
        }
    }
    return params

def get_attached_rs_devices():
    devices_from_ctx = rs.context().query_devices()
    serial_pattern = r'N: (\d{12})'
    usb_pattern = r'USB.{3}'
    devices = []

    for device in devices_from_ctx:
        match = re.search(serial_pattern, str(device))
        usb_type = re.search(usb_pattern, str(device)).group()
        if match and usb_type in ["USB3.2", "USB3.1", "USB3.0"]:
            devices.append((match.group(1), usb_type))

    if not devices:
        print("No RealSense devices found.")
    return devices

if __name__ == '__main__':
    params = load_camera_params()
    attached_devices = get_attached_rs_devices()

    if not attached_devices:
        print("No RealSense devices available. Exiting.")
    else:
        cameras = [RS_Camera(params, serial, cxn) for serial, cxn in attached_devices]

        for camera in cameras:
            camera.start()

        try:
            for idx, camera in enumerate(cameras):
                color_image, depth_image = camera.read()
                if color_image is not None and depth_image is not None:
                    cv2.imshow(f"Camera {idx} - Color", color_image)
                    depth_image_scaled = cv2.convertScaleAbs(depth_image, alpha=0.3)
                    cv2.imshow(f"Camera {idx} - Depth", depth_image_scaled)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            for camera in cameras:
                camera.stop()
            cv2.destroyAllWindows()