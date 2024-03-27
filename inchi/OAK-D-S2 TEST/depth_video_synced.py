import depthai as dai
import numpy as np
import cv2
import time

from datetime import timedelta



pipeline = dai.Pipeline()
pipeline.setXLinkChunkSize(0)

 # Device name



monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)


sync = pipeline.create(dai.node.Sync)

xoutGrp = pipeline.create(dai.node.XLinkOut)

xoutGrp.setStreamName("xout")

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
monoLeft.setCamera("left")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
monoRight.setCamera("right")
monoLeft.setFps(110)
monoRight.setFps(110)


sync.setSyncThreshold(timedelta(milliseconds=10))

monoLeft.out.link(sync.inputs["left"])
monoRight.out.link(sync.inputs["right"])


sync.out.link(xoutGrp.input)


delay=0
fpsCounter = 0
fps=0
timeCounter = time.time()
with dai.Device(pipeline) as device:
    print('Device name:', device.getDeviceName())
    # Bootloader version
    if device.getBootloaderVersion() is not None:
        print('Bootloader version:', device.getBootloaderVersion())
    # Print out usb speed
        print('Usb speed:', device.getUsbSpeed().name)
    # Connected cameras
        print('Connected cameras:', device.getConnectedCameraFeatures())
    queue = device.getOutputQueue("xout", 10, False)
    #fpsCounter["xout"] = fpsCounter.get("xout", 0.0) + len(messages)
    while True:
        msgGrp = queue.get()
        for name, msg in msgGrp:
            
            t1 = time.time()
            fpsCounter=fpsCounter+1
            frame2 = msg.getCvFrame()
            frame2 = cv2.cvtColor(frame2, cv2.COLOR_GRAY2BGR)
            cv2.putText(frame2, "delay: {:.2f}".format(delay), (100, 100), cv2.FONT_HERSHEY_TRIPLEX, 2, (0,255,0) , 2 , cv2.LINE_AA)
            cv2.putText(frame2, "fps: {:.2f}".format(fps), (100, 200), cv2.FONT_HERSHEY_TRIPLEX, 2, (0,255,0) , 2 , cv2.LINE_AA)
            cv2.imshow(name, frame2)
            delay=(time.time()-t1)*1000
            if time.time() - timeCounter > 1:
               # node.warn(f'FPS: {fpsCounter}')
                sla=time.time()-timeCounter
                fps=fpsCounter/sla/2
                fpsCounter = 0
                timeCounter = time.time()
               # print("fps",fps)
        if cv2.waitKey(1) == ord("q"):
            break

                        
       
