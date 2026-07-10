from __future__ import annotations

from typing import Any, Dict


class SafetyMonitor:
    def __init__(self, danger_distance_m: float = 1.5) -> None:
        self._danger_distance_m = danger_distance_m

    def is_dangerous(self, result: Dict[str, Any]) -> bool:
        detections = result.get("detections", [])
        if not isinstance(detections, list):
            return False

        for item in detections:
            if not isinstance(item, dict):
                continue
            distance = item.get("distance_m")
            label = str(item.get("label", ""))
            if isinstance(distance, (int, float)) and distance <= self._danger_distance_m:
                if label in {"person", "pedestrian", "car", "bicycle", "motorcycle", "obstacle"}:
                    return True
        return False
