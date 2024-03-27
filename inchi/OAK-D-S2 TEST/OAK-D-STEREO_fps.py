import cv2
import depthai as dai
import time
from datetime import timedelta
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
    print('Connected cameras:', device.getConnectedCameraFeatures())

    # Create pipeline
    pipeline = dai.Pipeline()
    cams = device.getConnectedCameraFeatures()
    pipeline.setXLinkChunkSize(0)
    streams = []
    for cam in cams:
        if str(cam.supportedTypes) == "[<CameraSensorType.MONO: 1>]":
            print(str(cam), str(cam.socket), cam.socket,cam.supportedTypes)
            c = pipeline.create(dai.node.MonoCamera)
            x = pipeline.create(dai.node.XLinkOut)
            c.out.link(x.input)
            c.setFps(100)
            c.setBoardSocket(cam.socket)
            c.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
            stream = str(cam.socket)
            if cam.name:
                stream = f'{cam.name} ({stream})'
            x.setStreamName(stream)
            streams.append(stream)
            
    print (streams)
   # xdepth =pipeline.create(dai.node.StereoDepth)
    # sync = pipeline.create(dai.node.Sync)
    # sync.setSyncThreshold(timedelta(milliseconds=12))
    # Start pipeline
    device.startPipeline(pipeline)
    
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
                    frame2 = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                    cv2.putText(frame2, "FPS: {:.2f}".format(fps), (200, 200), cv2.FONT_HERSHEY_TRIPLEX, 2, (0,255,0) , 2 , cv2.LINE_AA)
                    
                    cv2.imshow(stream, frame2)
                    if fps<80:
                        print ('low fps',"FPS: {:.2f}".format(fps),time.time())
                        
        if time.time() - tfps >= 1.0:
            scale = time.time() - tfps
            for stream in fpsCounter.keys():
                lastFpsCount[stream] = fpsCounter[stream] / scale
            fpsCounter = {}
            tfps = time.time()

        if cv2.waitKey(1) == ord('q'):
            break