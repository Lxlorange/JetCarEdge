from __future__ import annotations

import json
import socketserver
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import Bool, Empty, String

from jetcar_edge.image_codec import ImageCodec
from jetcar_edge.models import VideoFrameUpload
from jetcar_edge.safety import SafetyMonitor
from jetcar_edge.sensor_buffer import SensorBuffer
from jetcar_edge.ws_client import CloudWsClient


class EdgeUploadNode(Node):
    def __init__(self) -> None:
        super().__init__("jetcar_edge_upload")

        self.declare_parameter("car_id", "car_001")
        self.declare_parameter("stream_id", "camera_front")
        self.declare_parameter("cloud_host", "192.168.137.1")
        self.declare_parameter("cloud_port", 8000)
        self.declare_parameter(
            "algorithm_ids",
            ["yolov5-manhole-detect", "yolov8-road-damage"],
        )
        self.declare_parameter(
            "cloud_url",
            "",
        )
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("ai_enable_topic", "/jetcar/ai_enable")
        self.declare_parameter("algorithm_control_topic", "/jetcar/algorithm_ids")
        self.declare_parameter("app_control_host", "0.0.0.0")
        self.declare_parameter("app_control_port", 6001)
        self.declare_parameter("snapshot_topic", "/jetcar/snapshot")
        self.declare_parameter("ai_result_topic", "/jetcar/ai_result")
        self.declare_parameter("emergency_stop_topic", "/jetcar/emergency_stop")
        self.declare_parameter("upload_fps", 5.0)
        self.declare_parameter("image_width", 640)
        self.declare_parameter("jpeg_quality", 70)
        self.declare_parameter("queue_size", 2)
        self.declare_parameter("danger_distance_m", 1.5)
        self.declare_parameter("reconnect_seconds", 2.0)

        self._car_id = str(self.get_parameter("car_id").value)
        self._stream_id = str(self.get_parameter("stream_id").value)
        self._algorithm_ids = self._read_algorithm_ids()
        self._upload_interval = 1.0 / max(float(self.get_parameter("upload_fps").value), 0.1)
        self._last_upload_at = 0.0
        self._upload_enabled = False
        self._snapshot_requested = False
        self._control_server = None
        self._control_thread = None

        self._codec = ImageCodec(
            target_width=int(self.get_parameter("image_width").value),
            jpeg_quality=int(self.get_parameter("jpeg_quality").value),
        )
        self._sensors = SensorBuffer()
        self._safety = SafetyMonitor(
            danger_distance_m=float(self.get_parameter("danger_distance_m").value),
        )

        self._result_pub = self.create_publisher(
            String,
            str(self.get_parameter("ai_result_topic").value),
            10,
        )
        self._emergency_pub = self.create_publisher(
            Bool,
            str(self.get_parameter("emergency_stop_topic").value),
            10,
        )

        self.create_subscription(
            Bool,
            str(self.get_parameter("ai_enable_topic").value),
            self._on_ai_enable,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("algorithm_control_topic").value),
            self._on_algorithm_ids,
            10,
        )
        self.create_subscription(
            Empty,
            str(self.get_parameter("snapshot_topic").value),
            self._on_snapshot,
            10,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._sensors.update_lidar,
            10,
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self._sensors.update_imu,
            10,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("camera_topic").value),
            self._on_image,
            10,
        )

        self._cloud = CloudWsClient(
            self._cloud_url(),
            queue_size=int(self.get_parameter("queue_size").value),
            reconnect_seconds=float(self.get_parameter("reconnect_seconds").value),
            expect_response=False,
            on_result=self._on_cloud_result,
            on_log=lambda msg: self.get_logger().info(msg),
        )
        self._start_control_server()
        self.get_logger().info(
            f"JetCar edge upload node started stream_id={self._stream_id} upload_enabled={self._upload_enabled}"
        )

    def destroy_node(self) -> bool:
        self._stop_control_server()
        self._cloud.stop()
        return super().destroy_node()

    def _on_ai_enable(self, msg: Bool) -> None:
        self._set_ai_enabled(bool(msg.data), reason="ros_ai_enable")

    def _on_snapshot(self, _msg: Empty) -> None:
        self._snapshot_requested = True
        self.get_logger().info("single-frame snapshot requested")

    def _on_algorithm_ids(self, msg: String) -> None:
        algorithms = self._parse_algorithm_text(msg.data)
        if not algorithms:
            self._set_ai_algorithms([], reason="ros_algorithm_ids_empty")
            return
        self._set_ai_algorithms(algorithms, reason="ros_algorithm_ids")

    def _set_ai_enabled(self, enabled: bool, *, reason: str) -> None:
        if self._upload_enabled == enabled:
            return
        self._upload_enabled = enabled
        if enabled:
            self._cloud.update_url(self._cloud_url())
            self._cloud.start()
        else:
            self._cloud.stop()
        self.get_logger().info(f"AI upload enabled={enabled} reason={reason}")

    def _set_ai_algorithms(self, algorithms: list[str], *, reason: str) -> None:
        if algorithms == self._algorithm_ids:
            self._set_ai_enabled(bool(algorithms), reason=reason)
            return
        self._algorithm_ids = algorithms
        if algorithms:
            self._cloud.update_url(self._cloud_url())
            self._set_ai_enabled(True, reason=reason)
        else:
            self._set_ai_enabled(False, reason=reason)
        self.get_logger().info(f"algorithm_ids updated: {','.join(self._algorithm_ids) or '<none>'}")

    def _on_app_control(self, payload: dict) -> dict:
        algorithms = self._algorithms_from_control(payload)
        self._set_ai_algorithms(algorithms, reason="app_control")
        return {
            "ok": True,
            "car_id": self._car_id,
            "stream_id": self._stream_id,
            "upload_enabled": self._upload_enabled,
            "algorithm_ids": self._algorithm_ids,
            "cloud_url": self._cloud_url() if self._algorithm_ids else "",
        }

    def _on_image(self, msg: Image) -> None:
        now = time.monotonic()
        due = now - self._last_upload_at >= self._upload_interval
        should_upload = self._upload_enabled and due
        if not should_upload and not self._snapshot_requested:
            return

        self._last_upload_at = now
        self._snapshot_requested = False

        try:
            encoded = self._codec.encode(msg)
            frame = VideoFrameUpload(
                car_id=self._car_id,
                image=encoded,
            )
            self._cloud.submit(frame.to_dict())
            self.get_logger().info(
                f"frame queued for cloud upload: {encoded.width}x{encoded.height}"
            )
        except Exception as exc:
            self.get_logger().warning(f"failed to process camera frame: {exc}")

    def _on_cloud_result(self, result: dict) -> None:
        text = json.dumps(result, ensure_ascii=False, separators=(",", ":"))
        self._result_pub.publish(String(data=text))

        dangerous = self._safety.is_dangerous(result)
        self._emergency_pub.publish(Bool(data=dangerous))
        if dangerous:
            self.get_logger().warning("dangerous object detected; emergency_stop=true")

    def _cloud_url(self) -> str:
        explicit = str(self.get_parameter("cloud_url").value).strip()
        if explicit:
            return explicit
        host = str(self.get_parameter("cloud_host").value).strip()
        port = int(self.get_parameter("cloud_port").value)
        algorithms = ",".join(self._algorithm_ids)
        return (
            f"ws://{host}:{port}/ws/video/{self._car_id}/{self._stream_id}/edge"
            f"?algorithm_ids={algorithms}&include_image=true"
        )

    def _start_control_server(self) -> None:
        host = str(self.get_parameter("app_control_host").value).strip()
        port = int(self.get_parameter("app_control_port").value)
        if port <= 0:
            self.get_logger().info("app control TCP server disabled")
            return
        node = self

        class Handler(socketserver.StreamRequestHandler):
            def handle(self) -> None:
                for raw in self.rfile:
                    line = raw.decode("utf-8", errors="replace").strip()
                    if not line:
                        continue
                    try:
                        payload = json.loads(line)
                        if not isinstance(payload, dict):
                            raise ValueError("control payload must be a JSON object")
                        result = node._on_app_control(payload)
                    except Exception as exc:
                        result = {"ok": False, "error": str(exc)}
                    self.wfile.write((json.dumps(result, ensure_ascii=False) + "\n").encode("utf-8"))
                    self.wfile.flush()

        class ThreadingServer(socketserver.ThreadingTCPServer):
            allow_reuse_address = True

        self._control_server = ThreadingServer((host, port), Handler)
        self._control_thread = threading.Thread(
            target=self._control_server.serve_forever,
            name="jetcar-app-ai-control",
            daemon=True,
        )
        self._control_thread.start()
        self.get_logger().info(f"app AI control TCP server listening on {host}:{port}")

    def _stop_control_server(self) -> None:
        if self._control_server is None:
            return
        self._control_server.shutdown()
        self._control_server.server_close()
        if self._control_thread is not None:
            self._control_thread.join(timeout=2.0)
        self._control_server = None
        self._control_thread = None

    def _read_algorithm_ids(self) -> list[str]:
        value = self.get_parameter("algorithm_ids").value
        if isinstance(value, str):
            items = value.split(",")
        else:
            items = list(value)
        algorithms = [str(item).strip() for item in items if str(item).strip()]
        return algorithms or ["yolov8-road-damage"]

    def _parse_algorithm_text(self, value: str) -> list[str]:
        text = value.strip()
        if not text:
            return []
        try:
            loaded = json.loads(text)
            if isinstance(loaded, list):
                return [str(item).strip() for item in loaded if str(item).strip()]
            if isinstance(loaded, dict):
                raw = loaded.get("algorithm_ids") or loaded.get("algorithms") or []
                if isinstance(raw, str):
                    return [item.strip() for item in raw.split(",") if item.strip()]
                return [str(item).strip() for item in raw if str(item).strip()]
        except json.JSONDecodeError:
            pass
        return [item.strip() for item in text.split(",") if item.strip()]

    def _algorithms_from_control(self, payload: dict) -> list[str]:
        raw_algorithms = payload.get("algorithm_ids") or payload.get("algorithms")
        if isinstance(raw_algorithms, str):
            return [item.strip() for item in raw_algorithms.split(",") if item.strip()]
        if isinstance(raw_algorithms, list):
            return [str(item).strip() for item in raw_algorithms if str(item).strip()]

        mode = str(payload.get("mode") or "").strip().lower()
        if mode in {"off", "stop", "idle", "none"}:
            return []
        if mode in {"similarity", "search"}:
            return ["yolov5-similarity"]

        mask = str(payload.get("mask") or "").strip().upper()
        if mask:
            if len(mask) < 2:
                raise ValueError("mask must contain at least two characters, for example TF or FF")
            algorithms = []
            if mask[0] == "T":
                algorithms.append("yolov5-manhole-detect")
            if mask[1] == "T":
                algorithms.append("yolov8-road-damage")
            return algorithms

        enabled = payload.get("enabled")
        if enabled is False:
            return []
        return self._algorithm_ids


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EdgeUploadNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
