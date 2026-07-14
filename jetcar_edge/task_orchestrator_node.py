from __future__ import annotations

import json
import socketserver
import threading
import time
from dataclasses import dataclass, field
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


INSPECTION_ALGORITHMS = ["yolov5-manhole-detect", "yolov8-road-damage"]
SIMILARITY_ALGORITHMS = ["yolov5-similarity"]


@dataclass
class TaskState:
    task_id: str = ""
    mode: str = "idle"
    status: str = "idle"
    message: str = ""
    active: bool = False
    started_at: float = 0.0
    updated_at: float = 0.0
    summary: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "type": "edge_task_state",
            "task_id": self.task_id,
            "mode": self.mode,
            "status": self.status,
            "message": self.message,
            "active": self.active,
            "started_at": self.started_at,
            "updated_at": self.updated_at,
            "summary": self.summary,
        }


class TaskOrchestratorNode(Node):
    """Phone-facing task switch for demo-first Edge workflows.

    This node intentionally does not call Nav2 or map localization. It only
    translates task commands from the phone into algorithm enable/disable
    messages. Motion for similarity search is handled by edge_upload_node's
    visual-servo controller from Cloud similarity results and lidar safety data.
    """

    def __init__(self) -> None:
        super().__init__("jetcar_edge_task_orchestrator")

        self.declare_parameter("task_control_host", "0.0.0.0")
        self.declare_parameter("task_control_port", 6002)
        self.declare_parameter("algorithm_control_topic", "/jetcar/algorithm_ids")
        self.declare_parameter("task_status_topic", "/jetcar/task_status")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("publish_status_seconds", 1.0)

        self._algorithm_pub = self.create_publisher(
            String,
            str(self.get_parameter("algorithm_control_topic").value),
            10,
        )
        self._status_pub = self.create_publisher(
            String,
            str(self.get_parameter("task_status_topic").value),
            10,
        )
        self._cmd_pub = self.create_publisher(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            10,
        )

        self._state = TaskState(updated_at=time.time())
        self._state_lock = threading.Lock()
        self._server = None
        self._server_thread = None

        self._start_control_server()
        self.create_timer(float(self.get_parameter("publish_status_seconds").value), self._publish_state)
        self.get_logger().info("JetCar simple task switch started")

    def destroy_node(self) -> bool:
        self._cancel_active_task("node_destroy")
        self._stop_control_server()
        return super().destroy_node()

    def _start_control_server(self) -> None:
        host = str(self.get_parameter("task_control_host").value).strip()
        port = int(self.get_parameter("task_control_port").value)
        if port <= 0:
            self.get_logger().info("task control TCP server disabled")
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
                            raise ValueError("task payload must be a JSON object")
                        result = node._handle_command(payload)
                    except Exception as exc:
                        result = {"ok": False, "error": str(exc)}
                    self.wfile.write((json.dumps(result, ensure_ascii=False) + "\n").encode("utf-8"))
                    self.wfile.flush()

        class ThreadingServer(socketserver.ThreadingTCPServer):
            allow_reuse_address = True

        self._server = ThreadingServer((host, port), Handler)
        self._server_thread = threading.Thread(
            target=self._server.serve_forever,
            name="jetcar-task-control",
            daemon=True,
        )
        self._server_thread.start()
        self.get_logger().info(f"task control TCP server listening on {host}:{port}")

    def _stop_control_server(self) -> None:
        if self._server is None:
            return
        self._server.shutdown()
        self._server.server_close()
        if self._server_thread is not None:
            self._server_thread.join(timeout=2.0)
        self._server = None
        self._server_thread = None

    def _handle_command(self, payload: dict[str, Any]) -> dict[str, Any]:
        mode = str(payload.get("mode") or payload.get("type") or "").strip().lower()
        if mode in {"status", "task_status"}:
            return {"ok": True, "state": self._snapshot_state()}
        if mode in {"stop", "stop_task", "cancel"}:
            self._cancel_active_task("app_stop")
            return {"ok": True, "state": self._snapshot_state()}
        if mode in {"inspection_task", "road_inspection_task"}:
            return self._start_task(
                mode="inspection_task",
                algorithms=INSPECTION_ALGORITHMS,
                message="road inspection started",
            )
        if mode in {"similarity_search_task", "search_task", "similarity", "search"}:
            return self._start_task(
                mode="similarity_search_task",
                algorithms=SIMILARITY_ALGORITHMS,
                message="visual similarity search started",
            )
        if mode == "navigate_to_point":
            raise ValueError("navigate_to_point is disabled; map navigation is no longer used")
        if mode in {"waypoints", "list_waypoints", "set_waypoints", "update_waypoints"}:
            raise ValueError("waypoint navigation is disabled; visual search does not use waypoints")
        raise ValueError(f"unsupported task mode: {mode}")

    def _start_task(self, *, mode: str, algorithms: list[str], message: str) -> dict[str, Any]:
        self._cancel_active_task("new_task")
        task_id = f"{mode}-{int(time.time() * 1000)}"
        now = time.time()
        with self._state_lock:
            self._state = TaskState(
                task_id=task_id,
                mode=mode,
                status="running",
                message=message,
                active=True,
                started_at=now,
                updated_at=now,
                summary={"algorithm_ids": list(algorithms)},
            )
        self._publish_algorithms(algorithms)
        self._publish_state()
        return {"ok": True, "task_id": task_id, "state": self._snapshot_state()}

    def _cancel_active_task(self, reason: str) -> None:
        self._publish_algorithms([])
        self._publish_stop()
        with self._state_lock:
            if self._state.active:
                self._state.status = "cancelled"
                self._state.message = reason
                self._state.active = False
                self._state.updated_at = time.time()
        self._publish_state()

    def _publish_algorithms(self, algorithms: list[str]) -> None:
        self._algorithm_pub.publish(String(data=json.dumps({"algorithm_ids": algorithms}, separators=(",", ":"))))

    def _publish_stop(self) -> None:
        self._cmd_pub.publish(Twist())

    def _publish_state(self) -> None:
        self._status_pub.publish(String(data=json.dumps(self._snapshot_state(), ensure_ascii=False, separators=(",", ":"))))

    def _snapshot_state(self) -> dict[str, Any]:
        with self._state_lock:
            return self._state.to_dict()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskOrchestratorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
