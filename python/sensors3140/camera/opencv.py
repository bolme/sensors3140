
import cscore as cs
import cv2
import threading
import time

class Camera:
    def __init__(self, camera_name, camera_id, width=1280, height=720, fps=30):
        self.camera_name = camera_name
        self.cap = cv2.VideoCapture(camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.frame_id = 0
        self.prev_time = time.time()
        self.last_frame = None

    def start_capture(self):
        threading.Thread(target=self._capture_thread, daemon=True).start()

    def _capture_thread(self):
        img = None
        while True:
            retval, img = self.cap.read(img)
            curr_time = time.time()
            print("FPS: ", 1/(curr_time - self.prev_time))
            self.prev_time = curr_time
            self.frame_id += 1
            self.last_frame = img

    def get_frame(self):
        return self.last_frame
    

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