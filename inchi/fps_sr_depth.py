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
            c.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
            x = pipeline.create(dai.node.XLinkOut)
            c.out.link(x.input)
            c.setBoardSocket(cam.socket)
            c.setFps(110)
            stream = str(cam.socket)


        if str(cam.socket) == "CameraBoardSocket.CAM_C":
            c = pipeline.create(dai.node.MonoCamera)
            c.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
            x = pipeline.create(dai.node.XLinkOut)
            c.out.link(x.input)
            c.setBoardSocket(cam.socket)
            c.setFps(90)
            stream = str(cam.socket)       
          
        
        
        
        if cam.name:
            stream = f'{cam.name} ({stream})'
        x.setStreamName(stream)
        streams.append(stream)
        
        print (stream)
 
    device.startPipeline(pipeline)



    color = (0, 255, 0)
    fpsCounter = {}
    lastFpsCount = {}
    tfps = time.time()
    while not device.isClosed():
        
#        queueleft = device.getQueueEvents("left (CameraBoardSocket.CAM_B)")
#        queueright =device.getQueueEvents("right (CameraBoardSocket.CAM_C)")
        
        messageslefts = device.getOutputQueue("left (CameraBoardSocket.CAM_B)").tryGetAll()
#        messagesright = device.getOutputQueue("right (CameraBoardSocket.CAM_C)").tryGetAll()    
        
        fpsCounter["left (CameraBoardSocket.CAM_B)"] = fpsCounter.get("left (CameraBoardSocket.CAM_B)", 0.0) + len(messageslefts)

        for messagesleft in messageslefts:    
                # Display arrived frames
            if type(messagesleft) == dai.ImgFrame:
                           
                fps = lastFpsCount.get("left (CameraBoardSocket.CAM_B)", 0)
                frame = messagesleft.getCvFrame()
                cv2.putText(frame, "Fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_DUPLEX , 2, color)
                cv2.imshow("left (CameraBoardSocket.CAM_B)", frame)
                print("HERE")
        if time.time() - tfps >= 1.0:
            scale = time.time() - tfps
            for stream in fpsCounter.keys():
                lastFpsCount[stream] = fpsCounter[stream] / scale
            fpsCounter = {}
            tfps = time.time()

        if cv2.waitKey(1) == ord('q'):
            break
