from __future__ import annotations

import math
import threading
from typing import Any, Dict, Optional

from sensor_msgs.msg import Imu, LaserScan


class SensorBuffer:
    def __init__(self, max_lidar_ranges: int = 720) -> None:
        self._lock = threading.Lock()
        self._max_lidar_ranges = max_lidar_ranges
        self._lidar: Optional[Dict[str, Any]] = None
        self._imu: Optional[Dict[str, Any]] = None

    def update_lidar(self, msg: LaserScan) -> None:
        ranges = [float(v) if math.isfinite(v) else None for v in msg.ranges]
        if self._max_lidar_ranges > 0 and len(ranges) > self._max_lidar_ranges:
            step = max(1, len(ranges) // self._max_lidar_ranges)
            ranges = ranges[::step]
            angle_increment = msg.angle_increment * step
        else:
            angle_increment = msg.angle_increment

        payload = {
            "stamp": _stamp_to_seconds(msg.header.stamp.sec, msg.header.stamp.nanosec),
            "angle_min": float(msg.angle_min),
            "angle_max": float(msg.angle_max),
            "angle_increment": float(angle_increment),
            "range_min": float(msg.range_min),
            "range_max": float(msg.range_max),
            "ranges": ranges,
        }
        with self._lock:
            self._lidar = payload

    def update_imu(self, msg: Imu) -> None:
        payload = {
            "stamp": _stamp_to_seconds(msg.header.stamp.sec, msg.header.stamp.nanosec),
            "orientation": [
                float(msg.orientation.x),
                float(msg.orientation.y),
                float(msg.orientation.z),
                float(msg.orientation.w),
            ],
            "angular_velocity": [
                float(msg.angular_velocity.x),
                float(msg.angular_velocity.y),
                float(msg.angular_velocity.z),
            ],
            "linear_acceleration": [
                float(msg.linear_acceleration.x),
                float(msg.linear_acceleration.y),
                float(msg.linear_acceleration.z),
            ],
        }
        with self._lock:
            self._imu = payload

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "lidar": self._lidar,
                "imu": self._imu,
            }

    def front_distance_m(self, half_angle_rad: float = 0.25) -> Optional[float]:
        with self._lock:
            lidar = self._lidar
        if not lidar:
            return None

        angle_min = float(lidar.get("angle_min", 0.0))
        angle_increment = float(lidar.get("angle_increment", 0.0))
        range_min = float(lidar.get("range_min", 0.0))
        range_max = float(lidar.get("range_max", 0.0))
        ranges = lidar.get("ranges") or []

        values = []
        for index, value in enumerate(ranges):
            if value is None:
                continue
            angle = angle_min + index * angle_increment
            if abs(angle) > half_angle_rad:
                continue
            distance = float(value)
            if range_min <= distance <= range_max:
                values.append(distance)
        if not values:
            return None
        return min(values)


def _stamp_to_seconds(sec: int, nanosec: int) -> float:
    return float(sec) + float(nanosec) / 1_000_000_000.0
