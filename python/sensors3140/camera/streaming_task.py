import cv2
import cscore as cs
from typing import Optional, Any
from sensors3140.camera.camera import FrameData
import time
from sensors3140.task_base import TaskBase, TaskPriority
from sensors3140.tables.network_tables import NetworkTablesManager
import sensors3140

CURRENT_PORT = 8081

class StreamingTask(TaskBase):
    '''Low Latency Streaming Task'''

    def __init__(self, sensor_id: str, priority: TaskPriority = TaskPriority.NORMAL, width: int = 320, height: int = 240, fps: int = 10):
        super().__init__(f"camera{sensor_id}", priority)
        self.width = width
        self.height = height
        self.fps = fps
        self.prev_frame = None

        # Initialize cscore components
        self.camera = cs.CvSource("cvsource", cs.VideoMode.PixelFormat.kMJPEG, 
                                width, height, fps)
        global CURRENT_PORT
        port = CURRENT_PORT
        CURRENT_PORT += 1
        
        self.mjpegServer = cs.MjpegServer("httpserver", port)
        self.mjpegServer.setSource(self.camera)

        self.table = NetworkTablesManager()
        
        
        self.port = port

        self.sensor_id = sensor_id

        self.update_table()

        self.tableupdatetime = time.time()

    def update_table(self):
        self.table.setString(f"sensors3140/streams/camera{self.sensor_id}/ip", sensors3140.get_ip_addresses()[0])
        self.table.setString(f"sensors3140/streams/camera{self.sensor_id}/host", sensors3140.get_hostname())
        self.table.setDouble(f"sensors3140/streams/camera{self.sensor_id}/port", self.port)
        self.table.setString(f"sensors3140/streams/camera{self.sensor_id}/url", f"http://{sensors3140.get_ip_addresses()[0]}:{self.port}/stream.mjpg")
        self.table.setDouble(f"sensors3140/streams/camera{self.sensor_id}/width", self.width)
        self.table.setDouble(f"sensors3140/streams/camera{self.sensor_id}/height", self.height)
        self.table.setDouble(f"sensors3140/streams/camera{self.sensor_id}/fps", self.fps)
        self.table.setDouble(f"sensors3140/streams/camera{self.sensor_id}/timestamp", time.time())
        self.table.setDouble(f"sensors3140/streams/camera{self.sensor_id}/frame_id", -1)

    def process(self, frame_data: Any) -> Optional[Any]:
        """Process incoming frames and stream only the most recent one"""
        frame_data: FrameData
        current_time = time.time()    

        if current_time - self.tableupdatetime >= 10:
            self.update_table()
            self.tableupdatetime = current_time

        self.table.setDouble(f"sensors3140/streams/camera{self.sensor_id}/timestamp", frame_data.timestamp)
        self.table.setDouble(f"sensors3140/streams/camera{self.sensor_id}/frame_id", frame_data.frame_id)


        if frame_data is None:
            return None

        # Empty the input queue to get the most recent frame
        while not self.input_queue.empty():
            frame_data = self.input_queue.get()

        # Drop frames to maintain target fps
        if self.prev_frame is not None:
            time_diff = current_time - self.prev_frame
            if time_diff < 1.0 / self.fps:
                return None

        self.prev_frame = frame_data.timestamp

        # Resize the frame if needed
        frame = frame_data.frame
        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))

        try:
            self.camera.putFrame(frame)
            self.table.setDouble(f"sensors3140/streams/camera{self.sensor_id}/latency", current_time - frame_data.timestamp)
            return True
        except Exception as e:
            self.logger.error(f"Error streaming frame: {e}")
            self.table.setDouble(f"sensors3140/streams/camera{self.sensor_id}/latency", -1.0)
            return False



    def stop(self):
        """Override stop to cleanup cscore resources"""
        super().stop()
        self.camera.close()
        self.mjpegServer.close()




    