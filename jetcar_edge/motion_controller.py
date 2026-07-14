from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional

from geometry_msgs.msg import Twist

from jetcar_edge.sensor_buffer import SensorBuffer


LogFn = Callable[[str], None]


@dataclass
class VisualServoConfig:
    enabled: bool = True
    cmd_vel_topic: str = "/cmd_vel"
    align_tolerance: float = 0.12
    target_stop_distance_m: float = 0.7
    safety_stop_distance_m: float = 0.35
    target_stop_bbox_area: float = 0.28
    lost_timeout_seconds: float = 3.0
    search_angular_z: float = 0.60
    step_search_enabled: bool = True
    search_step_degrees: float = 45.0
    search_step_jitter_degrees: float = 10.0
    search_forward_after_degrees: float = 330.0
    search_forward_linear_x: float = 0.08
    search_forward_seconds: float = 0.8
    search_settle_seconds: float = 0.35
    search_result_timeout_seconds: float = 3.0
    align_angular_gain: float = 0.8
    approach_linear_x: float = 0.08
    cautious_approach_linear_x: float = 0.045
    approach_step_seconds: float = 0.45
    approach_angular_gain: float = 0.45
    command_timeout_seconds: float = 0.8


class VisualServoController:
    def __init__(
        self,
        *,
        config: VisualServoConfig,
        sensor_buffer: SensorBuffer,
        cmd_pub,
        on_log: Optional[LogFn] = None,
    ) -> None:
        self._config = config
        self._sensors = sensor_buffer
        self._cmd_pub = cmd_pub
        self._on_log = on_log or (lambda _msg: None)
        self._active = False
        self._state = "idle"
        self._last_target_at = 0.0
        self._last_command_at = 0.0
        self._state_started_at = 0.0
        self._last_result_at = 0.0
        self._current_search_step_seconds = 0.0
        self._search_degrees_since_forward = 0.0
        self._current_motion_seconds = 0.0

    @property
    def active(self) -> bool:
        return self._active

    @property
    def state(self) -> str:
        return self._state

    def start(self) -> None:
        if not self._config.enabled:
            self._on_log("visual servo disabled; motion commands will not be published")
            return
        self._active = True
        self._state = "waiting_result" if self._config.step_search_enabled else "searching"
        self._last_target_at = 0.0
        self._last_result_at = 0.0
        self._search_degrees_since_forward = 0.0
        self._current_motion_seconds = 0.0
        self._state_started_at = time.monotonic()
        if self._config.step_search_enabled:
            self._publish_stop()
        else:
            self._publish(0.0, self._config.search_angular_z)

    def stop(self, reason: str = "stopped") -> None:
        if not self._active and self._state == "idle":
            return
        self._publish_stop()
        self._active = False
        self._state = "idle"
        self._on_log(f"visual servo stopped reason={reason}")

    def handle_similarity_result(self, result: dict[str, Any]) -> dict[str, Any]:
        if not self._active or not self._config.enabled:
            return {"motion_state": self._state, "command": "disabled"}

        matched = _as_bool(result.get("matched"))
        similarity = _as_float(result.get("similarity"), 0.0)
        center_norm = _as_float_pair(result.get("center_norm"))
        bbox_area = _bbox_area(result.get("bbox_norm"))
        front_distance = self._sensors.front_distance_m()
        self._last_result_at = time.monotonic()

        if front_distance is not None and front_distance <= self._config.safety_stop_distance_m:
            self._state = "safety_stop"
            self._publish_stop()
            return {
                "motion_state": self._state,
                "command": "stop",
                "reason": "front_obstacle_too_close",
                "front_distance_m": front_distance,
                "similarity": similarity,
            }

        if not matched or center_norm is None:
            return self._handle_no_match(front_distance=front_distance, similarity=similarity)

        self._last_target_at = time.monotonic()
        x_error = center_norm[0] - 0.5
        if abs(x_error) > self._config.align_tolerance:
            angular_z = -x_error * self._config.align_angular_gain
            self._state = "aligning"
            self._publish(0.0, angular_z)
            return {
                "motion_state": self._state,
                "command": "align",
                "center_norm": center_norm,
                "x_error": round(x_error, 4),
                "angular_z": round(angular_z, 4),
                "front_distance_m": front_distance,
                "bbox_area": bbox_area,
                "similarity": similarity,
            }

        if bbox_area is not None and bbox_area >= self._config.target_stop_bbox_area:
            self._state = "arrived"
            self._publish_stop()
            return {
                "motion_state": self._state,
                "command": "stop",
                "reason": "target_bbox_large_enough",
                "center_norm": center_norm,
                "bbox_area": round(bbox_area, 4),
                "similarity": similarity,
            }

        if front_distance is not None and front_distance <= self._config.target_stop_distance_m:
            self._state = "arrived"
            self._publish_stop()
            return {
                "motion_state": self._state,
                "command": "stop",
                "reason": "target_distance_reached",
                "center_norm": center_norm,
                "front_distance_m": front_distance,
                "bbox_area": bbox_area,
                "similarity": similarity,
            }

        if front_distance is None:
            angular_z = -x_error * self._config.approach_angular_gain
            return self._begin_approach_step(
                speed=self._config.cautious_approach_linear_x,
                angular_z=angular_z,
                reason="front_distance_unavailable_use_cautious_step",
                center_norm=center_norm,
                x_error=x_error,
                bbox_area=bbox_area,
                similarity=similarity,
                front_distance=front_distance,
            )

        angular_z = -x_error * self._config.approach_angular_gain
        return self._begin_approach_step(
            speed=self._config.approach_linear_x,
            angular_z=angular_z,
            reason="target_aligned",
            center_norm=center_norm,
            x_error=x_error,
            bbox_area=bbox_area,
            similarity=similarity,
            front_distance=front_distance,
        )

    def _begin_approach_step(
        self,
        *,
        speed: float,
        angular_z: float,
        reason: str,
        center_norm: list[float],
        x_error: float,
        bbox_area: float | None,
        similarity: float,
        front_distance: float | None,
    ) -> dict[str, Any]:
        self._state = "approach_forward"
        self._state_started_at = time.monotonic()
        self._current_motion_seconds = self._config.approach_step_seconds
        self._publish(speed, angular_z)
        payload = {
            "motion_state": self._state,
            "command": "approach_step",
            "reason": reason,
            "center_norm": center_norm,
            "x_error": round(x_error, 4),
            "linear_x": speed,
            "angular_z": round(angular_z, 4),
            "duration_seconds": self._config.approach_step_seconds,
            "bbox_area": bbox_area,
            "similarity": similarity,
        }
        if front_distance is not None:
            payload["front_distance_m"] = front_distance
        return payload

    def tick(self) -> dict[str, Any] | None:
        if not self._active or not self._config.enabled:
            return None
        now = time.monotonic()
        if (
            self._last_target_at > 0.0
            and now - self._last_target_at > self._config.lost_timeout_seconds
        ):
            self._state = "waiting_result" if self._config.step_search_enabled else "searching"
            self._last_target_at = 0.0
            self._state_started_at = now
            if self._config.step_search_enabled:
                self._publish_stop()
                return {"motion_state": self._state, "command": "wait_result", "reason": "target_lost"}
            self._publish(0.0, self._config.search_angular_z)
            return {"motion_state": self._state, "command": "search", "reason": "target_lost"}
        if self._config.step_search_enabled:
            return self._tick_step_search(now)
        if now - self._last_command_at > self._config.command_timeout_seconds:
            if self._state == "searching":
                self._publish(0.0, self._config.search_angular_z)
            elif self._state in {"aligning", "approaching"}:
                self._publish_stop()
                return {"motion_state": self._state, "command": "stop", "reason": "command_timeout"}
        return None

    def _handle_no_match(self, *, front_distance: float | None, similarity: float) -> dict[str, Any]:
        if not self._config.step_search_enabled:
            self._state = "searching"
            self._publish(0.0, self._config.search_angular_z)
            return {
                "motion_state": self._state,
                "command": "search",
                "angular_z": self._config.search_angular_z,
                "front_distance_m": front_distance,
                "similarity": similarity,
            }

        now = time.monotonic()
        if self._state in {"step_rotating", "settling"}:
            return {
                "motion_state": self._state,
                "command": "ignore_result_while_moving",
                "front_distance_m": front_distance,
                "similarity": similarity,
            }

        motion = self._next_search_action(now, reason="target_not_matched", front_distance=front_distance)
        motion["front_distance_m"] = front_distance
        motion["similarity"] = similarity
        return motion

    def _tick_step_search(self, now: float) -> dict[str, Any] | None:
        if self._state == "step_rotating":
            if now - self._state_started_at >= self._current_search_step_seconds:
                return self._begin_settling(now, reason="step_complete")
            if now - self._last_command_at > self._config.command_timeout_seconds:
                self._publish(0.0, self._config.search_angular_z)
            return None

        if self._state == "search_forward":
            if now - self._state_started_at >= self._current_motion_seconds:
                return self._begin_settling(now, reason="search_forward_complete")
            if now - self._last_command_at > self._config.command_timeout_seconds:
                self._publish(self._config.search_forward_linear_x, 0.0)
            return None

        if self._state == "approach_forward":
            if now - self._state_started_at >= self._current_motion_seconds:
                return self._begin_settling(now, reason="approach_step_complete")
            if now - self._last_command_at > self._config.command_timeout_seconds:
                self._publish(self._config.cautious_approach_linear_x, 0.0)
            return None

        if self._state == "settling":
            if now - self._state_started_at >= self._config.search_settle_seconds:
                self._state = "waiting_result"
                self._state_started_at = now
                self._publish_stop()
                return {
                    "motion_state": self._state,
                    "command": "wait_result",
                    "reason": "settled",
                }
            return None

        if self._state == "waiting_result":
            last = max(self._last_result_at, self._state_started_at)
            if now - last >= self._config.search_result_timeout_seconds:
                return self._next_search_action(
                    now,
                    reason="result_timeout",
                    front_distance=self._sensors.front_distance_m(),
                )
        return None

    def _begin_settling(self, now: float, *, reason: str) -> dict[str, Any]:
        self._state = "settling"
        self._state_started_at = now
        self._current_motion_seconds = 0.0
        self._publish_stop()
        return {
            "motion_state": self._state,
            "command": "stop",
            "reason": reason,
        }

    def _next_search_action(
        self,
        now: float,
        *,
        reason: str,
        front_distance: float | None,
    ) -> dict[str, Any]:
        if self._search_degrees_since_forward >= self._config.search_forward_after_degrees:
            if front_distance is None or front_distance > self._config.target_stop_distance_m:
                return self._begin_search_forward(now, reason=reason, front_distance=front_distance)
            self._search_degrees_since_forward = 0.0
            motion = self._begin_search_step(now, reason=f"{reason}_front_blocked")
            motion["front_distance_m"] = front_distance
            return motion
        return self._begin_search_step(now, reason=reason)

    def _begin_search_forward(
        self,
        now: float,
        *,
        reason: str,
        front_distance: float | None,
    ) -> dict[str, Any]:
        self._state = "search_forward"
        self._state_started_at = now
        self._current_motion_seconds = self._config.search_forward_seconds
        self._search_degrees_since_forward = 0.0
        self._publish(self._config.search_forward_linear_x, 0.0)
        payload = {
            "motion_state": self._state,
            "command": "search_forward_step",
            "reason": reason,
            "linear_x": self._config.search_forward_linear_x,
            "duration_seconds": self._config.search_forward_seconds,
        }
        if front_distance is not None:
            payload["front_distance_m"] = front_distance
        return payload

    def _begin_search_step(self, now: float, *, reason: str) -> dict[str, Any]:
        degrees = self._next_search_step_degrees()
        angular_z = self._config.search_angular_z
        self._current_search_step_seconds = math.radians(degrees) / max(abs(angular_z), 1e-3)
        self._search_degrees_since_forward += abs(degrees)
        self._state = "step_rotating"
        self._state_started_at = now
        self._publish(0.0, angular_z)
        return {
            "motion_state": self._state,
            "command": "rotate_step",
            "reason": reason,
            "angular_z": angular_z,
            "target_degrees": round(degrees, 2),
            "duration_seconds": round(self._current_search_step_seconds, 3),
        }

    def _next_search_step_degrees(self) -> float:
        base = max(1.0, float(self._config.search_step_degrees))
        jitter = max(0.0, float(self._config.search_step_jitter_degrees))
        if jitter <= 0.0:
            return base
        return max(1.0, base + random.uniform(-jitter, jitter))

    def _publish(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self._cmd_pub.publish(msg)
        self._last_command_at = time.monotonic()

    def _publish_stop(self) -> None:
        self._publish(0.0, 0.0)


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"true", "1", "yes", "matched", "found"}
    if isinstance(value, (int, float)):
        return value != 0
    return False


def _as_float(value: Any, default: float) -> float:
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        try:
            return float(value)
        except ValueError:
            return default
    return default


def _as_float_pair(value: Any) -> list[float] | None:
    if not isinstance(value, (list, tuple)) or len(value) < 2:
        return None
    x = _as_float(value[0], -1.0)
    y = _as_float(value[1], -1.0)
    if x < 0.0 or y < 0.0:
        return None
    return [max(0.0, min(1.0, x)), max(0.0, min(1.0, y))]


def _bbox_area(value: Any) -> float | None:
    if not isinstance(value, (list, tuple)) or len(value) < 4:
        return None
    x1 = _as_float(value[0], -1.0)
    y1 = _as_float(value[1], -1.0)
    x2 = _as_float(value[2], -1.0)
    y2 = _as_float(value[3], -1.0)
    if min(x1, y1, x2, y2) < 0.0:
        return None
    width = max(0.0, min(1.0, x2) - max(0.0, x1))
    height = max(0.0, min(1.0, y2) - max(0.0, y1))
    return width * height
