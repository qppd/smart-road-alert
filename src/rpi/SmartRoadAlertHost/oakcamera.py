import depthai as dai
import cv2

# Create pipeline
pipeline = dai.Pipeline()

# Define camera using v3 API — .build() is required
cam = pipeline.create(dai.node.Camera).build()

# Request output at desired size (replaces setPreviewSize + linking)
videoOut = cam.requestOutput((640, 480), type=dai.ImgFrame.Type.BGR888p)

# Create output queue directly on the output (no XLinkOut node needed)
q = videoOut.createOutputQueue()

# Start pipeline (replaces `with dai.Device(pipeline)`)
pipeline.start()

while pipeline.isRunning():
    frame = q.get().getCvFrame()
    cv2.imshow("Camera", frame)
    if cv2.waitKey(1) == ord('q'):
        pipeline.stop()
        break

cv2.destroyAllWindows()