#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np
import time
# import depthai_sdk.classes as imu
# Connect to device and start pipeline





with dai.Device() as device:
    # Device name
    print('Device name:', device.getDeviceName())
    # Bootloader version
    if device.getBootloaderVersion() is not None:
        print('Bootloader version:', device.getBootloaderVersion())
    # Print out usb speed
    print('Usb speed:', device.getUsbSpeed().name)
    # Connected cameras
    #print('Connected cameras:', device.getConnectedCameraFeatures())
    
      # Create pipeline
    pipeline = dai.Pipeline()
    depthout= pipeline.create(dai.node.XLinkOut)
    cdepth=pipeline.create(dai.node.StereoDepth)
    cams = device.getConnectedCameraFeatures()
    streams = []
    for cam in cams:
        print(str(cam), str(cam.socket), cam.socket)
        print
        # if str(cam.socket)== "CameraBoardSocket.CAM_A" :
        #     c = pipeline.create(dai.node.Camera)
        #     c.setBoardSocket(dai.CameraBoardSocket.CAM_A)
            
        #     x = pipeline.create(dai.node.XLinkOut)
            
        #     tof = pipeline.create(dai.node.ToF)
        #     # Configure the ToF node
        #     tofConfig = tof.initialConfig.get()
        #     # tofConfig.depthParams.freqModUsed = dai.RawToFConfig.DepthParams.TypeFMod.MIN
        #     tofConfig.depthParams.freqModUsed = dai.RawToFConfig.DepthParams.TypeFMod.MAX
        #     tofConfig.depthParams.avgPhaseShuffle = False
        #     tofConfig.depthParams.minimumAmplitude = 3.0
        #     tof.initialConfig.set(tofConfig)

        #     c.raw.link(tof.input)
        #     tof.depth.link(x.input)
        #     c.setBoardSocket(cam.socket)
        #     stream = str(cam.socket)




        if str(cam.socket) == "CameraBoardSocket.CAM_B":
            c = pipeline.create(dai.node.MonoCamera)
            c.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
            x = pipeline.create(dai.node.XLinkOut)
            c.out.link(x.input)
            c.out.link(cdepth.left)
            c.setBoardSocket(cam.socket)
            c.setFps(110)
            stream = str(cam.socket)


        if str(cam.socket) == "CameraBoardSocket.CAM_C":
            c = pipeline.create(dai.node.MonoCamera)
            c.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
            x = pipeline.create(dai.node.XLinkOut)
            c.out.link(x.input)
            c.out.link(cdepth.right)
            c.setBoardSocket(cam.socket)
            c.setFps(90)
            stream = str(cam.socket)       
          
        
        
        
        if cam.name:
            stream = f'{cam.name} ({stream})'
        x.setStreamName(stream)
        streams.append(stream)
        
        print (stream)
    cdepth.depth.link(depthout.input)
    depthout.setStreamName("depth") 
    dai.RawCameraControl.AutoFocusMode.OFF
    dai.CameraControl.AutoFocusMode.OFF
    # Closer-in minimum depth, disparity range is doubled (from 95 to 190):
    extended_disparity = False
    # Better accuracy for longer distance, fractional disparity 32-levels:
    subpixel = False
    # Better handling for occlusions:
    lr_check = False
        # Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
    cdepth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    # Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
    cdepth.initialConfig.setMedianFilter(dai.MedianFilter.MEDIAN_OFF)
    cdepth.setLeftRightCheck(lr_check)
    cdepth.setExtendedDisparity(extended_disparity)
    cdepth.setSubpixel(subpixel)
    device.startPipeline(pipeline)
    


    color = (0, 255, 0)
    fpsCounter = {}
    lastFpsCount = {}
    tfps = time.time()
    while not device.isClosed():
        
        queueleft  = device.getQueueEvents("CAMB-2L (CameraBoardSocket.CAM_B)")
        queueright = device.getQueueEvents("CAMC-2L (CameraBoardSocket.CAM_C)")
        q = device.getOutputQueue(name="depth", maxSize=4, blocking=False)

        messageslefts = device.getOutputQueue("CAMB-2L (CameraBoardSocket.CAM_B)").tryGetAll()
        messagesright = device.getOutputQueue("CAMC-2L (CameraBoardSocket.CAM_C)").tryGetAll()    


        inDisparity = q.get()  # blocking call, will wait until a new data has arrived
        depthFrame= inDisparity.getCvFrame()
        depth_downscaled = depthFrame[::4]
        if np.all(depth_downscaled == 0):
            min_depth = 0  # Set a default minimum depth value when all elements are zero
        else:
            min_depth = np.percentile(depth_downscaled[depth_downscaled != 0], 1)
        max_depth = np.percentile(depth_downscaled, 99)
        depthFrameColor = np.interp(depthFrame, (min_depth, max_depth), (0, 255)).astype(np.uint8)
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

        cv2.imshow("depth", depthFrameColor )

        

        fpsCounter["CAMB-2L (CameraBoardSocket.CAM_B)"] = fpsCounter.get("CAMB-2L (CameraBoardSocket.CAM_B)", 0.0) + len(messageslefts)

        for messagesleft in messageslefts:    
                # Display arrived frames
            if type(messagesleft) == dai.ImgFrame:
                           
                fps = lastFpsCount.get("CAMB-2L (CameraBoardSocket.CAM_B)", 0)
                frame = messagesleft.getCvFrame()
                
                cv2.putText(frame, "Fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_DUPLEX , 3, color)
                cv2.imshow("CAMB-2L (CameraBoardSocket.CAM_B)", frame)
                
                #print("HERE")
        if time.time() - tfps >= 1.0:
            scale = time.time() - tfps
            for stream in fpsCounter.keys():
                lastFpsCount[stream] = fpsCounter[stream] / scale
            fpsCounter = {}
            tfps = time.time()

        if cv2.waitKey(1) == ord('q'):
            break
