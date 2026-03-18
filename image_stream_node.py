import threading
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

@dataclass
class FramePacket:
    frame_id: int
    stamp_sec: float
    frame: any

class ImageStreamNode(Node):
    """
    - 订阅图像话题
    - 将最新帧缓存
    - 外部通过 get_latest_frame() 获取
    """

    def __init__(self, topic_name: str = "/camera/image_raw"):
        super().__init__("image_stream_node")

        self.bridge = CvBridge()
        self.topic_name = topic_name

        self._lock = threading.Lock()
        self._latest_packet: Optional[FramePacket] = None
        self._frame_id = 0
        self._last_consumed_id = -1

        self.subscription = self.create_subscription(
            Image,
            self.topic_name,
            self.image_callback,
            10,
        )

        self.get_logger().info(f"订阅图像话题: {self.topic_name}")

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge 转换失败: {e}")
            return

        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

        with self._lock:
            self._latest_packet = FramePacket(
                frame_id=self._frame_id,
                stamp_sec=stamp_sec,
                frame=frame
            )
            self._frame_id += 1

    def get_latest_frame(self, only_new: bool = True) -> Tuple[bool, Optional[any], Optional[int], Optional[float]]:
        """
        返回:
            ok, frame, frame_id, stamp_sec

        only_new=True:
            只返回还没消费过的新帧，避免重复处理同一帧
        """
        with self._lock:
            if self._latest_packet is None:
                return False, None, None, None

            if only_new and self._latest_packet.frame_id == self._last_consumed_id:
                return False, None, None, None

            pkt = self._latest_packet
            self._last_consumed_id = pkt.frame_id

            # copy 一份，避免外部修改缓存
            return True, pkt.frame.copy(), pkt.frame_id, pkt.stamp_sec