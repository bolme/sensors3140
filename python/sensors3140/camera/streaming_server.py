import time
import cv2
import cscore as cs
from sensors3140 import Camera

class StreamingServer:

    def __init__(self, width, height, fps, port=8081):
        self.width = width
        self.height = height
        self.fps = fps

        self.camera = cs.CvSource("cvsource", cs.VideoMode.PixelFormat.kMJPEG, width, height, fps)

        self.mjpegServer = cs.MjpegServer("httpserver", port)
        self.mjpegServer.setSource(self.camera)

    def putFrame(self,frame):

        # Resize the frame if needed
        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))

        self.camera.putFrame(frame)

if __name__ == "__main__":
    # Start the camera
    camera = Camera("camera", 1)
    camera.start_capture()

    # Start the streaming server
    server = StreamingServer(640, 480, 30)

    while True:
        frame = camera.get_frame()
        if frame is not None:
            server.putFrame(frame)
        time.sleep(0.01)