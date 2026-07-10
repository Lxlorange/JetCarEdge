from __future__ import annotations

import base64

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from jetcar_edge.models import EncodedImage


class ImageCodec:
    def __init__(self, target_width: int = 640, jpeg_quality: int = 70) -> None:
        self._bridge = CvBridge()
        self._target_width = target_width
        self._jpeg_quality = int(max(1, min(100, jpeg_quality)))

    def encode(self, msg: Image) -> EncodedImage:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        height, width = frame.shape[:2]

        if self._target_width > 0 and width > self._target_width:
            scale = self._target_width / float(width)
            target_size = (self._target_width, int(height * scale))
            frame = cv2.resize(frame, target_size, interpolation=cv2.INTER_AREA)

        ok, encoded = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality],
        )
        if not ok:
            raise RuntimeError("failed to encode camera frame as JPEG")

        out_height, out_width = frame.shape[:2]
        data = base64.b64encode(encoded.tobytes()).decode("ascii")
        return EncodedImage(width=out_width, height=out_height, data=data)

