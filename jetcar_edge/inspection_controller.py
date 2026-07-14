from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional

from geometry_msgs.msg import Twist
from std_msgs.msg import String

from jetcar_edge.sensor_buffer import SensorBuffer


LogFn = Callable[[str], None]
StopAiFn = Callable[[str], None]
ReportEventFn = Callable[[str, Dict[str, Any]], None]


INSPECTION_ALGORITHMS = {"yolov5-manhole-detect", "yolov8-road-damage"}


@dataclass
class InspectionConfig:
    enabled: bool = True
    target_count: int = 3
    linear_x: float = 0.07
    angular_z: float = 0.10
    sway_seconds: float = 1.8
    result_cooldown_seconds: float = 1.4
    detection_min_confidence: float = 0.25
    safety_stop_distance_m: float = 0.35
    obstacle_turn_angular_z: float = 0.35
    command_timeout_seconds: float = 0.6


class InspectionController:
    """Demo-first road inspection motion loop.

    The controller intentionally avoids Nav2. While inspection algorithms are
    active it drives forward slowly with a small alternating turn, counts cloud
    detection result frames, and stops after enough effective results.
    """

    def __init__(
        self,
        *,
        car_id: str,
        stream_id: str,
        config: InspectionConfig,
        sensor_buffer: SensorBuffer,
        cmd_pub,
        result_pub,
        stop_ai: StopAiFn,
        report_event: Optional[ReportEventFn] = None,
        on_log: Optional[LogFn] = None,
    ) -> None:
        self._car_id = car_id
        self._stream_id = stream_id
        self._config = config
        self._sensors = sensor_buffer
        self._cmd_pub = cmd_pub
        self._result_pub = result_pub
        self._stop_ai = stop_ai
        self._report_event = report_event
        self._on_log = on_log or (lambda _msg: None)
        self._active = False
        self._state = "idle"
        self._started_at = 0.0
        self._last_command_at = 0.0
        self._last_counted_at = 0.0
        self._count = 0
        self._last_result: Dict[str, Any] | None = None

    @property
    def active(self) -> bool:
        return self._active

    @property
    def state(self) -> str:
        return self._state

    @property
    def count(self) -> int:
        return self._count

    def start(self) -> None:
        if not self._config.enabled:
            self._on_log("inspection controller disabled; motion commands will not be published")
            return
        if self._active:
            return
        self._active = True
        self._state = "running"
        self._started_at = time.monotonic()
        self._last_command_at = 0.0
        self._last_counted_at = 0.0
        self._count = 0
        self._last_result = None
        self._publish_event(
            "inspection_started",
            {
                "count": self._count,
                "target_count": self._config.target_count,
                "motion": self._motion_payload("start"),
            },
        )
        self._on_log("road inspection controller started")

    def stop(self, reason: str = "stopped") -> None:
        if not self._active:
            self._state = "idle"
            return
        self._active = False
        self._state = "idle"
        self._publish_stop()
        self._publish_event(
            "inspection_stopped",
            {
                "reason": reason,
                "count": self._count,
                "target_count": self._config.target_count,
            },
        )
        self._on_log(f"road inspection controller stopped reason={reason}")

    def handle_cloud_result(self, message: Dict[str, Any]) -> None:
        if not self._active:
            return
        if message.get("type") != "algorithm_result":
            return
        algorithm_id = str(message.get("algorithm_id") or "")
        if algorithm_id not in INSPECTION_ALGORITHMS:
            return
        if str(message.get("car_id") or "") != self._car_id:
            return
        incoming_stream = str(message.get("stream_id") or "")
        if incoming_stream and incoming_stream != self._stream_id:
            return

        result = message.get("result")
        if not isinstance(result, dict):
            result = {}
        detections = _valid_detections(result, self._config.detection_min_confidence)
        if not detections:
            return

        now = time.monotonic()
        if now - self._last_counted_at < self._config.result_cooldown_seconds:
            return
        self._last_counted_at = now
        self._count += 1
        self._last_result = message

        payload = {
            "algorithm_id": algorithm_id,
            "count": self._count,
            "target_count": self._config.target_count,
            "detection_count": len(detections),
            "detections": detections[:5],
            "latency_ms": message.get("latency_ms"),
            "motion": self._motion_payload("detection"),
        }
        annotated_image = message.get("annotated_image")
        if annotated_image:
            payload["final_image"] = annotated_image
        self._publish_event("inspection_detection", payload)
        self._on_log(
            f"road inspection detection counted {self._count}/{self._config.target_count} "
            f"algorithm={algorithm_id} detections={len(detections)}"
        )

        if self._count >= max(1, int(self._config.target_count)):
            self._complete("target_count_reached")

    def tick(self) -> None:
        if not self._active or not self._config.enabled:
            return
        now = time.monotonic()
        if now - self._last_command_at <= self._config.command_timeout_seconds:
            return

        front_distance = self._sensors.front_distance_m()
        if front_distance is not None and front_distance <= self._config.safety_stop_distance_m:
            self._state = "obstacle_turn"
            direction = 1.0 if int((now - self._started_at) / 1.2) % 2 == 0 else -1.0
            self._publish(0.0, direction * abs(self._config.obstacle_turn_angular_z))
            self._publish_event(
                "inspection_warning",
                {
                    "reason": "front_obstacle_too_close",
                    "front_distance_m": front_distance,
                    "motion": self._motion_payload("obstacle_turn"),
                },
            )
            return

        self._state = "running"
        elapsed = max(0.0, now - self._started_at)
        sway_seconds = max(0.2, float(self._config.sway_seconds))
        wave = math.sin((elapsed / sway_seconds) * math.tau)
        angular_z = float(self._config.angular_z) * wave
        self._publish(float(self._config.linear_x), angular_z)

    def _complete(self, reason: str) -> None:
        self._active = False
        self._state = "complete"
        self._publish_stop()
        payload = {
            "reason": reason,
            "count": self._count,
            "target_count": self._config.target_count,
            "motion": self._motion_payload("stop"),
        }
        if self._last_result and self._last_result.get("annotated_image"):
            payload["final_image"] = self._last_result.get("annotated_image")
        self._publish_event("inspection_complete", payload)
        self._stop_ai("inspection_complete")
        self._on_log(f"road inspection complete count={self._count}")

    def _motion_payload(self, command: str) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "motion_state": self._state,
            "command": command,
            "linear_x": self._config.linear_x,
            "angular_z": self._config.angular_z,
        }
        front_distance = self._sensors.front_distance_m()
        if front_distance is not None:
            payload["front_distance_m"] = front_distance
        return payload

    def _publish(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self._cmd_pub.publish(msg)
        self._last_command_at = time.monotonic()

    def _publish_stop(self) -> None:
        self._publish(0.0, 0.0)

    def _publish_event(self, event: str, payload: Dict[str, Any]) -> None:
        message = {
            "type": "edge_road_inspection",
            "event": event,
            "car_id": self._car_id,
            "stream_id": self._stream_id,
            "state": self._state,
            "active": self._active,
            "started_at": self._started_at,
            **payload,
        }
        self._result_pub.publish(String(data=json.dumps(message, ensure_ascii=False, separators=(",", ":"))))
        if self._report_event is not None:
            try:
                self._report_event(event, message)
            except Exception as exc:
                self._on_log(f"failed to report edge event {event}: {exc}")


def _valid_detections(result: dict[str, Any], min_confidence: float) -> list[dict[str, Any]]:
    raw = result.get("detections")
    if not isinstance(raw, list):
        count = _as_int(result.get("detection_count"), 0)
        return [{} for _ in range(max(0, count))]

    detections = []
    for item in raw:
        if not isinstance(item, dict):
            continue
        confidence = _as_float(item.get("confidence"), 1.0)
        if confidence < min_confidence:
            continue
        detections.append(
            {
                key: value
                for key, value in item.items()
                if key in {"class_name", "confidence", "bbox_xyxy"}
            }
        )
    return detections


def _as_float(value: Any, default: float) -> float:
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        try:
            return float(value)
        except ValueError:
            return default
    return default


def _as_int(value: Any, default: int) -> int:
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        return int(value)
    if isinstance(value, str):
        try:
            return int(value)
        except ValueError:
            return default
    return default
