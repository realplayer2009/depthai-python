
# !/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np
import time

# # dai.RawCameraControl.AutoFocusMode.OFF
# # dai.CameraControl.AutoFocusMode.OFF
# # Closer-in minimum depth, disparity range is doubled (from 95 to 190):
# extended_disparity = False
# # Better accuracy for longer distance, fractional disparity 32-levels:
# subpixel = False
# # Better handling for occlusions:
# lr_check = True






# Connect to device and   start pipeline

 
    # print('Device name:', device.getDeviceName())
    # # Bootloader version
    # if device.getBootloaderVersion() is not None:
    #     print('Bootloader version:', device.getBootloaderVersion())
    # print('Usb speed:', device.getUsbSpeed().name)
    # # Connected cameras
    # print('Connected cameras:', device.getConnectedCameraFeatures())
    
    # cams = device.getConnectedCameraFeatures()
    # # streams = []
    # for cam in cams:
    #     print(str(cam), str(cam.socket), cam.socket)

        # c = pipeline.create(dai.node.Camera)
        # x = pipeline.create(dai.node.XLinkOut)
        # c.isp.link(x.input)
        # c.setBoardSocket(cam.socket)
        # stream = str(cam.socket)
        # if cam.name:
        #     stream = f'{cam.name} ({stream})'
        # x.setStreamName(stream)
        # streams.append(stream)
    # Create pipeline
pipeline = dai.Pipeline()
    #dai.RawCameraControl.AutoFocusMode=0



    # Define sources and outputs
    # monoLeft = pipeline.create(dai.node.MonoCamera)
    # monoRight = pipeline.create(dai.node.MonoCamera)
rgb=pipeline.create(dai.node.ColorCamera)
    
    #depth = pipeline.create(dai.node.StereoDepth)

    # xoutLeft = pipeline.create(dai.node.XLinkOut)
    # xoutRight = pipeline.create(dai.node.XLinkOut)
xouteRgb = pipeline.create(dai.node.XLinkOut)
    #xoutdepth = pipeline.create(dai.node.XLinkOut)

    # xoutLeft.setStreamName("left")
    # xoutRight.setStreamName("right")
xouteRgb.setStreamName("rgb")
    # xoutdepth.setStreamName("disparity")

    # Properties
    
    # monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    # monoLeft.setCamera("left")
    

    # monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    # monoRight.setCamera("right")
    # monoLeft.setFps(100)
    # monoRight.setFps(100)
rgb.setCamera("rgb")
#rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
rgb.preview.link(xouteRgb.input)
    # rgb.setFps(60)
    # #rgb.setIspScale()
    # rgb.setVideoSize(640, 480)

    # # Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
    # depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    # # Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
    # depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_3x3)
    # depth.setLeftRightCheck(lr_check)
    # depth.setExtendedDisparity(extended_disparity)
    # depth.setSubpixel(subpixel)

    # Linking

    # monoLeft.out.link(depth.left)
    # monoRight.out.link(depth.right)
   


    

    # monoLeft.out.link(xoutLeft.input)
    # monoRight.out.link(xoutRight.input)
    #depth.disparity.link(xoutdepth .input)

diffs = np.array([])

    # Output queue will be used to get the disparity frames from the outputs defined above
    
#    disparityQueue= device.getOutputQueue(name="disparity", maxSize=4, blocking=False)
   
    # LeftQueue = device.getOutputQueue(name=xoutLeft.getStreamName(), maxSize=4, blocking=False)
    # RightQueue = device.getOutputQueue(name=xoutRight.getStreamName(), maxSize=4 ,blocking=False)
with dai.Device() as device:    
    qrgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    RgbFrame=None
    LeftFrame=None
    RightFrame=None


    color = (0, 255, 0)
    startTime = time.monotonic()
    counter = 0
    fps = 0
   
    rgbstartTime = time.monotonic()
    rgbcounter = 0
    rgbfps = 0
     
    print (xouteRgb.getStreamName,"minergb")

    while True:
        
        vRgb=qrgb.get()
        
#        disparity = disparityQueue.get()  # blocking call, will wait until a new data has arrived


        RgbFrame=vRgb.getFrame()
        # LeftFrame=Left.getFrame()
        # RightFrame=Right.getFrame()
#        disparityframe = disparity.getFrame()
        # cLeftFrame=cv2.cvtColor(LeftFrame,cv2.cv2.COLOR_GRAY2BGR)
        # cRightFrame=cv2.cvtColor(RightFrame,cv2.cv2.COLOR_GRAY2BGR)
        



        fpsCounter = {}
        lastFpsCount = {}
        tfps = time.time()


 #      print('Latency: {:.2f} ms, Average latency: {:.2f} ms, Std: {:.2f}'.format(latencyMs, np.average(diffs), np.std(diffs)))
        cv2.putText(RgbFrame, "Fps: {:.2f}".format(rgbfps), (2, RgbFrame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, (255,255,255))
        cv2.putText(RgbFrame,'Latency: {:.2f} ms, Average latency: {:.2f} ms, Std: {:.2f}'.format(latencyMs, np.average(diffs), np.std(diffs)) , (10, 10), cv2.FONT_HERSHEY_TRIPLEX, 0.4,(255, 255, 255))
        cv2.imshow("RGB",RgbFrame)
#         cv2.putText(cLeftFrame, "Fps: {:.2f}".format(fps), (2, cLeftFrame.shape[0] - 4), cv2.FONT_HERSHEY_SIMPLEX, 2,color,2)
#  #       cv2.putText(cLeftFrame,'Latency: {:.2f} ms, Average latency: {:.2f} ms, Std: {:.2f}'.format(latencyMs, np.average(diffs), np.std(diffs)) , (10, 20), cv2.FONT_ITALIC, 0.6,color,1)
#         cv2.imshow("Left",cLeftFrame)
#         cv2.putText(cRightFrame, "Fps: {:.2f}".format(fps), (2, cRightFrame.shape[0] - 4), cv2.FONT_HERSHEY_SIMPLEX, 2, color,2)
#         cv2.imshow("Right",cRightFrame)
        
        
    #     # Normalization for better visualization
    #     disparityframe = (disparityframe * (255 / depth.initialConfig.getMaxDisparity())).astype(np.uint8)
    #  #   cv2.putText(frame, "Fps: {:.2f}".format(fps), (10, 10), cv2.FONT_HERSHEY_TRIPLEX, 0.4,(255, 255, 255))
    #     cv2.imshow("disparity", disparityframe)z

    #     # Available color maps: https://docs.opencv.org/3.4/d3/d50/group__imgproc__colormap.html
    #     disparityframe = cv2.applyColorMap(disparityframe, cv2.COLORMAP_JET)
    #     cv2.imshow("disparity_color", disparityframe)

       


        if cv2.waitKey(1) == ord('q'):
            break
