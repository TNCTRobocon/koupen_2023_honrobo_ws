## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.04), cv2.COLORMAP_JET)
        
        cvt_depth = cv2.cvtColor(depth_colormap,cv2.COLOR_BGR2GRAY)
        ret2 ,thresh_img = cv2.threshold(cvt_depth,0,255,cv2.THRESH_OTSU)
        print(ret2)
        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape
        thresh_img_dim = thresh_img.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            #images = np.hstack((resized_color_image, depth_colormap))
        else:
            resized_color_image = color_image
        if thresh_img_dim != depth_colormap_dim:
            resized_thresh_img = cv2.resize(thresh_img,dsize=(depth_colormap_dim[1],depth_colormap_dim[0]),interpolation=cv2.INTER_AREA)
        else:
            #images = np.hstack((color_image, depth_colormap))
            resized_thresh_img = thresh_img

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', resized_color_image)
        cv2.namedWindow('RealSense2', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense2', depth_colormap)
        cv2.namedWindow('RealSense3', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense3', resized_thresh_img)
        cv2.namedWindow('RealSense4', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense4', cvt_depth)
        lastkey = cv2.waitKey(1)

        if lastkey==ord("q"):
            break

finally:

    # Stop streaming
    print("stop streaming.")
    pipeline.stop()