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
            c = pipeline.create(dai.node.ColorCamera)
            c.setBoardSocket(dai.CameraBoardSocket.CAM_A)
            c.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            x = pipeline.create(dai.node.XLinkOut)
            c.preview.link(x.input)
            stream = str(cam.socket)
            c.setFps(60)
            print ("CAM-A ONLINE")
        else:
            c = pipeline.create(dai.node.MonoCamera)
            c.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            x = pipeline.create(dai.node.XLinkOut)
            c.out.link(x.input)
            c.setBoardSocket(cam.socket)
            stream = str(cam.socket)
            c.setFps(90)



        # if str(cam.socket) == "CameraBoardSocket.CAM_C":
        #     c = pipeline.create(dai.node.MonoCamera)
        #     c.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        #     x = pipeline.create(dai.node.XLinkOut)
        #     c.out.link(x.input)
        #     c.setBoardSocket(cam.socket)
        #     stream = str(cam.socket) 
        #     c.setFps(90)      
          
        
        
        
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
        
        queueNames = device.getQueueEvents(streams)
        for stream in queueNames:
            messages = device.getOutputQueue(stream).tryGetAll()
            fpsCounter[stream] = fpsCounter.get(stream, 0.0) + len(messages)
            for message in messages:
                # Display arrived frames
                if type(message) == dai.ImgFrame:
                    # render fps
                                        
                    fps = lastFpsCount.get(stream, 0)
                    frame = message.getCvFrame()
                    if str(stream)!="color (CameraBoardSocket.CAM_A)":
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
