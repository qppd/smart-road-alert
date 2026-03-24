import depthai as dai
import cv2

# Create pipeline
pipeline = dai.Pipeline()

# Use Camera node (replaces deprecated ColorCamera)
cam = pipeline.create(dai.node.Camera)
cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)  # RGB camera

# Output
xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("video")

# Use video output instead of preview, and specify size via OutputControl
cam.video.link(xout.input)

# Connect to device
with dai.Device(pipeline) as device:
    q = device.getOutputQueue(name="video", maxSize=4, blocking=False)
    while True:
        frame = q.get().getCvFrame()
        frame = cv2.resize(frame, (640, 480))  # Resize after capture
        cv2.imshow("Camera", frame)
        if cv2.waitKey(1) == ord('q'):
            break