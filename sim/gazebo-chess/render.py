"""相机画面：订阅 Gazebo 俯视相机（经 image_bridge 桥到 ROS 的 sensor_msgs/Image），
缓存最近一帧，并按需编码成 JPEG/PNG，供世界的 /perceive（喂大脑）和 /stream（人类页）用。

不自己起 executor——帧由世界的专职 ROS spin 线程持续送达（见 world.py 的 _spin_forever）。
"""
from __future__ import annotations

import numpy as np
from sensor_msgs.msg import Image

import config


class CameraFeed:
    """挂在某个 rclpy 节点上，订阅一路相机话题、缓存最近一帧（RGB numpy）。
    多相机 = 多个 CameraFeed，一路一个话题（见 config.cam_names/cam_topic），绝不共用。"""

    def __init__(self, node, topic: str) -> None:
        self._frame: np.ndarray | None = None
        node.create_subscription(Image, topic, self._on_image, 5)

    def _on_image(self, msg: Image) -> None:
        try:
            arr = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, -1)
            self._frame = arr[:, :, :3]   # 丢掉可能的 alpha，留 RGB
        except Exception:  # noqa: BLE001
            pass

    @property
    def frame(self) -> np.ndarray | None:
        return self._frame


def to_jpeg(frame: np.ndarray | None, quality: int = 80) -> bytes | None:
    """RGB numpy → JPEG 字节（cv2 要 BGR，转一下）。无帧返回 None。"""
    if frame is None:
        return None
    import cv2
    ok, buf = cv2.imencode(".jpg", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR),
                           [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    return buf.tobytes() if ok else None


def to_png(frame: np.ndarray | None) -> bytes | None:
    if frame is None:
        return None
    import cv2
    ok, buf = cv2.imencode(".png", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    return buf.tobytes() if ok else None
