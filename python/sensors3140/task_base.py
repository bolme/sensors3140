# Each task runs as its own thread.  THere is an input queue and output queue.  
# The thread runs its own loop and waits for new items to be added to the 
# queue and then processes them.
 
import threading
import queue
import logging
import os
import enum
from typing import Optional, Any
from abc import ABC, abstractmethod

class TaskPriority(enum.IntEnum):
    LOWEST = 19
    LOW = 10
    NORMAL = 0
    HIGH = -10
    HIGHEST = -19  # Requires root privileges

class TaskBase(ABC):

    def __init__(self, name: str, priority: TaskPriority = TaskPriority.NORMAL):
        self.name = name
        self.priority = priority
        self.input_queue = queue.Queue()
        self.output_queue = queue.Queue()
        self.running = False
        self.thread = None
        self.logger = logging.getLogger(name)
        self.max_queue_length = 30


    def add_input(self, data: Any):
        if self.input_queue.qsize() < self.max_queue_length:
            self.input_queue.put(data)
        else:
            self.logger.warning(f"Input queue for {self.name} is full")


    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()
        self.logger.info(f"Started {self.name} task")


    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        self.logger.info(f"Stopped {self.name} task")


    def _set_thread_priority(self):
        try:
            os.sched_setpriority(0, self.priority)
            self.logger.info(f"Set {self.name} priority to {self.priority}")
        except PermissionError:
            self.logger.warning(f"Permission denied setting priority {self.priority}")
        except Exception as e:
            self.logger.error(f"Failed to set priority: {e}")


    def _run_loop(self):
        self._set_thread_priority()
        while self.running:
            try:
                # Wait for input with timeout to allow checking running flag
                data = self.input_queue.get(timeout=0.1)
                #print(f"Processing {self.name} with data {data}")
                self.process(data)
                
            except queue.Empty:
                continue
            except Exception as e:
                self.logger.error(f"Error processing {self.name} data: {e}")


    @abstractmethod
    def process(self, data: Any) -> Optional[Any]:
        """Process input data and return result in self.output_queue"""
        pass

