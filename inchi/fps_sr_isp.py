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
    
    cams = device.getConnectedCameraFeatures()
    streams = []
    for cam in cams:
        print(str(cam), str(cam.socket), cam.socket)
        print
        if str(cam.socket)== "CameraBoardSocket.CAM_A" :
            c = pipeline.create(dai.node.Camera)
            c.setBoardSocket(dai.CameraBoardSocket.CAM_A)
            
            x = pipeline.create(dai.node.XLinkOut)
            
            tof = pipeline.create(dai.node.ToF)
            # Configure the ToF node
            tofConfig = tof.initialConfig.get()
            # tofConfig.depthParams.freqModUsed = dai.RawToFConfig.DepthParams.TypeFMod.MIN
            tofConfig.depthParams.freqModUsed = dai.RawToFConfig.DepthParams.TypeFMod.MAX
            tofConfig.depthParams.avgPhaseShuffle = False
            tofConfig.depthParams.minimumAmplitude = 3.0
            tof.initialConfig.set(tofConfig)

            c.raw.link(tof.input)
            tof.depth.link(x.input)
            c.setBoardSocket(cam.socket)
            stream = str(cam.socket)




        if str(cam.socket) == "CameraBoardSocket.CAM_B":
            c = pipeline.create(dai.node.MonoCamera)
            c.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            x = pipeline.create(dai.node.XLinkOut)
            c.out.link(x.input)
            c.setBoardSocket(cam.socket)
            stream = str(cam.socket)


        if str(cam.socket) == "CameraBoardSocket.CAM_C":
            c = pipeline.create(dai.node.MonoCamera)
            c.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            x = pipeline.create(dai.node.XLinkOut)
            c.out.link(x.input)
            c.setBoardSocket(cam.socket)
            stream = str(cam.socket)       
          
        
        
        
        if cam.name:
            stream = f'{cam.name} ({stream})'
        x.setStreamName(stream)
        streams.append(stream)
        c.setFps(90)
 
    device.startPipeline(pipeline)



    color = (0, 255, 0)
    fpsCounter = {}
    lastFpsCount = {}
    tfps = time.time()
    while not device.isClosed():
        queueNames = device.getQueueEvents(streams)
        for stream in queueNames:
            messages = device.getOutputQueue(stream).tryGetAll()
            fpsCounter[stream] = fpsCounter.get(stream, 0.0) + len(messages)
            for message in messages:
                # Display arrived frames
                if type(message) == dai.ImgFrame:
                    # render fps
                    if str(stream)=="tof (CameraBoardSocket.CAM_A)":


                        fps = lastFpsCount.get(stream, 0)
                        frame = message.getCvFrame()

                                                # Colorize the depth frame to jet colormap
                        depth_downscaled = frame[::4]
                        non_zero_depth = depth_downscaled[depth_downscaled != 0] # Remove invalid depth values
                        if len(non_zero_depth) == 0:
                            min_depth, max_depth = 0, 0
                        else:
                            min_depth = np.percentile(non_zero_depth, 3)
                            max_depth = np.percentile(non_zero_depth, 97)
                        depth_colorized = np.interp(frame, (min_depth, max_depth), (0, 255)).astype(np.uint8)
                        depth_colorized = cv2.applyColorMap(depth_colorized, cv2.COLORMAP_MAGMA)

                                
                        cv2.putText(depth_colorized, "Fps: {:.2f}".format(fps), (2, depth_colorized.shape[0] - 4), cv2.FONT_HERSHEY_DUPLEX , 2, color)
                                                
                        cv2.imshow(stream,  depth_colorized)                        


                    else:
                        
                        fps = lastFpsCount.get(stream, 0)
                        frame = message.getCvFrame()
                        cv2.putText(frame, "Fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_DUPLEX , 2, color)
                        cv2.imshow(stream, frame)
        if time.time() - tfps >= 1.0:
            scale = time.time() - tfps
            for stream in fpsCounter.keys():
                lastFpsCount[stream] = fpsCounter[stream] / scale
            fpsCounter = {}
            tfps = time.time()

        if cv2.waitKey(1) == ord('q'):
            break
