import depthai as dai
import cv2

# Create pipeline
pipeline = dai.Pipeline()

# Define camera (use Camera node to avoid deprecation warning)
cam = pipeline.create(dai.node.Camera)
cam.setPreviewSize(640, 480)
cam.setInterleaved(False)

# Output — fixed: use pipeline.create() instead of createXLinkOut()
xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("video")
cam.preview.link(xout.input)

# Connect to device
with dai.Device(pipeline) as device:
    q = device.getOutputQueue(name="video", maxSize=4, blocking=False)
    while True:
        frame = q.get().getCvFrame()
        cv2.imshow("Camera", frame)
        if cv2.waitKey(1) == ord('q'):
            break