import cv2
import cscore as cs
from typing import Optional, Any
from sensors3140.camera.camera import FrameData
import time
from sensors3140.task_base import TaskBase, TaskPriority
from sensors3140.tables.network_tables import NetworkTablesManager
import sensors3140
import logging
import traceback

# Track the currently assigned port to avoid port conflicts between multiple streams
CURRENT_PORT = 8081

class StreamingTask(TaskBase):
    '''
    Low Latency Streaming Task - Handles streaming camera frames via MJPEG server
    This task efficiently streams frames by dropping old frames to maintain target FPS
    and publishes stream information to NetworkTables for clients to discover
    '''

    def __init__(self, sensor_id: str, priority: TaskPriority = TaskPriority.NORMAL, width: int = 320, height: int = 240, fps: int = 10):
        super().__init__(f"camera{sensor_id}", priority)
        self.width = width
        self.height = height
        self.fps = fps
        self.prev_frame = None
        self.camera_name = f"camera{sensor_id}"

        # Initialize cscore components for MJPEG streaming
        self.camera = cs.CvSource(self.camera_name, cs.VideoMode.PixelFormat.kMJPEG, 
                                width, height, fps)
        global CURRENT_PORT
        port = CURRENT_PORT
        CURRENT_PORT += 1
        
        self.mjpegServer = cs.MjpegServer("httpserver", port)
        self.mjpegServer.setSource(self.camera)

        self.table = NetworkTablesManager()
        
        self.port = port
        self.sensor_id = sensor_id

        # For periodic logging
        self.last_log_time = time.time()
        self.frames_streamed = 0
        self.frames_dropped = 0

        # Initialize stream info in NetworkTables
        self.update_table()

        # Track when we last updated the table
        self.tableupdatetime = time.time()
        
        self.logger.info(f"Created streaming task for camera{sensor_id} on port {port} ({width}x{height} @ {fps} FPS)")

    def update_table(self):
        """
        Update NetworkTables with stream information for clients to discover
        Publishes IP, hostname, port, URL, and stream parameters
        """
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
        """
        Process incoming frames and stream only the most recent one
        
        Optimizes streaming performance by:
        1. Emptying the queue to get the most recent frame
        2. Dropping frames to maintain target FPS
        3. Resizing frames if needed
        4. Publishing stream performance metrics to NetworkTables
        
        Args:
            frame_data: FrameData object containing the camera frame and metadata
            
        Returns:
            True if frame was streamed successfully, False on error, None if frame was dropped
        """
        frame_data: FrameData
        current_time = time.time()    

        # Update NetworkTables info periodically (every 10 seconds)
        if current_time - self.tableupdatetime >= 10:
            self.update_table()
            self.tableupdatetime = current_time
            
            # Log streaming statistics every 10 seconds
            if self.frames_streamed > 0 or self.frames_dropped > 0:
                frame_size_mb = (self.width * self.height * 3) / (1024 * 1024)
                self.logger.info(
                    f"Stream camera{self.sensor_id}: Streamed {self.frames_streamed} frames, "
                    f"dropped {self.frames_dropped} frames, "
                    f"size: {self.width}x{self.height} ({frame_size_mb:.2f}MB), "
                    f"target FPS: {self.fps:.1f}"
                )
                self.frames_streamed = 0
                self.frames_dropped = 0

        # Update the current frame timestamp and ID in NetworkTables
        self.table.setDouble(f"sensors3140/streams/camera{self.sensor_id}/timestamp", frame_data.timestamp)
        self.table.setDouble(f"sensors3140/streams/camera{self.sensor_id}/frame_id", frame_data.frame_id)

        if frame_data is None:
            return None

        # Empty the input queue to get the most recent frame
        # This helps reduce latency by skipping older frames
        queue_dropped = 0
        while not self.input_queue.empty():
            frame_data = self.input_queue.get()
            queue_dropped += 1
            
        if queue_dropped > 1:
            self.logger.debug(f"Stream camera{self.sensor_id}: Dropped {queue_dropped-1} frames from queue")
            self.frames_dropped += queue_dropped - 1

        # Rate limiting - drop frames to maintain target FPS
        if self.prev_frame is not None:
            time_diff = current_time - self.prev_frame
            if time_diff < 1.0 / self.fps:
                self.frames_dropped += 1
                return None

        self.prev_frame = frame_data.timestamp

        # Resize the frame if needed to match stream dimensions
        frame = frame_data.frame
        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))

        try:
            # Push the frame to the MJPEG stream
            self.camera.putFrame(frame)
            # Record latency metrics
            self.table.setDouble(f"sensors3140/streams/camera{self.sensor_id}/latency", current_time - frame_data.timestamp)
            self.frames_streamed += 1
            return True
        except Exception as e:
            traceback.print_exc()
            self.logger.error(f"Error streaming frame: {e}")
            self.table.setDouble(f"sensors3140/streams/camera{self.sensor_id}/latency", -1.0)
            return False

    def stop(self):
        """
        Override stop to cleanup cscore resources
        Properly closes the camera source and MJPEG server
        """
        self.logger.info(f"Stopping stream for camera{self.sensor_id}")
        super().stop()
        
        # Free resources correctly - CvSource doesn't have close()
        cs.CameraServer.removeCamera(self.camera_name)




