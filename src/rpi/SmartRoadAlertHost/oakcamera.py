import depthai as dai
import cv2

# Create pipeline
pipeline = dai.Pipeline()

# Define camera (v3 API)
cam = pipeline.create(dai.node.Camera)
cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)  # RGB camera

# Output
xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("video")

# In v3, use requestOutput to set size and link
cam.requestOutput((640, 480)).link(xout.input)

# Connect to device
with dai.Device(pipeline) as device:
    q = device.getOutputQueue(name="video", maxSize=4, blocking=False)
    while True:
        frame = q.get().getCvFrame()
        cv2.imshow("Camera", frame)
        if cv2.waitKey(1) == ord('q'):
            break