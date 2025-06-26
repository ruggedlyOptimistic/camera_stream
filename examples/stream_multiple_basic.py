import pyrealsense2 as rs
import cv2
import numpy as np

# Configure multiple RealSense streams
pipeline1 = rs.pipeline()
config1 = rs.config()
config1.enable_device('309622300248')
config1.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline2 = rs.pipeline()
config2 = rs.config()
config2.enable_device('311322301090')
config2.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline1.start(config1)
pipeline2.start(config2)

while True:
    frames1 = pipeline1.wait_for_frames()
    depth_frame1 = frames1.get_depth_frame()
    depth_image1 = np.asanyarray(depth_frame1.get_data())

    frames2 = pipeline2.wait_for_frames()
    depth_frame2 = frames2.get_depth_frame()
    depth_image2 = np.asanyarray(depth_frame2.get_data())

    cv2.imshow('Camera 1 Depth', depth_image1)
    cv2.imshow('Camera 2 Depth', depth_image2)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

pipeline1.stop()
pipeline2.stop()
cv2.destroyAllWindows()