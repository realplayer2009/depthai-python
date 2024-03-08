import pyrealsense2 as rs
import numpy as np
import cv2
import depthai as dai
import pyrealsense2 as rs

if __name__ == "__main__":
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 1024,768, rs.format.z16, 30)
    config.enable_stream(rs.stream.infrared,1024,768,rs.format.y8,30)
#    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    # Start streaming
    pipeline.start(config)
    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
 #           color_frame = frames.get_color_frame()
            infrared_frame= frames.get_infrared_frame()
#            if not depth_frame or not color_frame:
            if not depth_frame or not infrared_frame:                         
                continue
            # Convert images to numpy arrays
 
            depth_image = np.asanyarray(depth_frame.get_data())
            infrared_image= np.asanyarray(infrared_frame.get_data())                            
#            color_image = np.asanyarray(color_frame.get_data())

 
            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.01), cv2.COLORMAP_JET)
            infrared_colormap=cv2.applyColorMap(cv2.convertScaleAbs(infrared_image, alpha=0.5), cv2.COLORMAP_JET)
            # Stack both images horizontally
#            images = np.hstack((color_image, depth_colormap))
            images = np.hstack((infrared_colormap,depth_colormap))
            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        # Stop streaming
        pipeline.stop()