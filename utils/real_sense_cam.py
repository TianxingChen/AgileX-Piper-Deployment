import pyrealsense2 as rs
import numpy as np
import threading
from collections import deque

class RealSenseCam:
    def __init__(self, serial_number, name):
        self.serial_number = serial_number
        self.name = name
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_device(serial_number)
        # 只启用彩色图像流
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # 使用双端队列替换队列，简化帧管理
        self.frame_buffer = deque(maxlen=1)  # 仅保留最新一帧
        self.keep_running = False
        self.thread = None
        self.exit_event = threading.Event()

    def start(self):
        self.keep_running = True
        self.exit_event.clear()
        self.pipeline.start(self.config)
        self.thread = threading.Thread(target=self._update_frames)
        self.thread.daemon = True  # 设置为守护线程
        self.thread.start()

    def _update_frames(self):
        try:
            while not self.exit_event.is_set():
                # 等待彩色帧数据（超时5秒）
                frames = self.pipeline.wait_for_frames(5000)
                color_frame = frames.get_color_frame()
                
                if color_frame:
                    # 转换为NumPy数组并存储
                    color_image = np.asanyarray(color_frame.get_data())[:, :, ::-1]
                    self.frame_buffer.append(color_image)  # 保留最新帧
        except Exception as e:
            print(f"Error from {self.name} camera: {e}")
        finally:
            self.pipeline.stop()

    def get_latest_image(self):
        if self.frame_buffer:
            return self.frame_buffer[-1]  # 返回最新一帧
        return None

    def stop(self):
        self.exit_event.set()
        self.keep_running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2)  # 等待线程安全退出
        self.pipeline.stop()