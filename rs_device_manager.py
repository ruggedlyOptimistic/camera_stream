import pyrealsense2 as rs
from rs_camera import RS_Camera
import re
import sys

class RS_Device_Manager:

    def __init__(self, params, dbg):   
        system_params = params['Cameras']  
        self.cameras = []
        self.camera_index = 0         

        try:
            # Get attached RealSense cameras
            self.cameras = self.load_from_usb(system_params)            
            
        except Exception as e:
            print("Fatal error with loading devices!")    # Get attached RealSense cameras
            
            print(e)
            print("Quitting process...")
            sys.exit(1)

    # Belongs to rs_device_manager
    def load_from_usb(self, system_params):
        print("Loading RealSense camera devices from USB...")
        devices_from_ctx = rs.context().query_devices()
        serial_pattern = r'N: (\d{12})'
        usb_pattern = r'USB.{3}'
        cameras = []

        # print("Configured serial numbers: ",end='')
        # for serial in system_params['serial']:
        #     print(serial)

        for device in devices_from_ctx:
            match = re.search(serial_pattern, str(device))
            usb_type = re.search(usb_pattern, str(device)).group()

            # print(f"USB Type: {usb_type}")

            if match:
                serial_number = match.group(1)

                if serial_number in system_params['serial']:
                    print(f"Serial number {serial_number} found in configuration file. Loading camera settings...")
                    # Create camera objects here
                    cameras.append(RS_Camera(system_params, serial=serial_number, cxn=usb_type))
                
                else:
                    print(f"Serial number {serial_number} not found in configuration file. Skipping...")


            if usb_type not in ["USB3.2", "USB3.1", "USB3.0"]:
                print(f"RealSense Device {match.group(1)} found with unsupported USB connection type! Skipping this device...")

        if len(cameras) == 0:
            print("No Real Sense Devices Found on USB!")

        return cameras

    def load_from_network(self):
        pass

    def reset_devices(self, cameras='all'):
        if cameras == 'all':
            cameras = self.cameras
        self.stop_devices(cameras)
        self.start_devices(cameras)

    def pause_devices(self, cameras):
        pass

    def start_devices(self, cameras='all'):
        if cameras == 'all':
            cameras = self.cameras

        for camera in cameras:
            camera.start()

    def stop_devices(self, cameras='all'):
        if cameras == 'all':
            cameras = self.cameras

        for camera in cameras:
            camera.stop()