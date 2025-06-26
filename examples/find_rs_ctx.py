import pyrealsense2 as rs
import re
import subprocess


# Find all RealSense Devices attached to the bus
def find_rs_devices(target_model=""):
    rs_devices = rs.context().query_devices()
                
    return rs_devices

if __name__ == '__main__':
    rs_devices = find_rs_devices()
    print(f"data type of list elements: {type(rs_devices[0])}")
    
    data_string = str(rs_devices[0])
    print(f"data as a string:  {data_string}")
    serial_pattern = r'N: (\d{12})'
    usb_pattern = r'USB.{3}'

    print("entering loop")
    for device in rs_devices:
        # device.group_dict()
        print(type(str(device)))
        match = re.search(serial_pattern, str(device))
        usb_type = re.search(usb_pattern, str(device)).group()
        if match: 
            print(f"USB type: {usb_type}")
            if usb_type in ["USB3.2", "USB3.1", "USB3.0"]:
                
                print(f"Device Serial : {match.group(1)}")
            else:
                print(f"Real Sense Device {match.group(1)} found with unsupported USB connection type! Skipping this device...")
