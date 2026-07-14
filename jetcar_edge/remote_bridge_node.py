from __future__ import annotations

import socket
import socketserver
import threading
import time
from dataclasses import dataclass
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Empty


@dataclass
class RemoteState:
    car_type: int = 1
    speed_xy: int = 100
    speed_z: int = 100
    stabilize: int = 0
    last_command_at: float = 0.0
    client: str = ""


class RemoteBridgeNode(Node):
    """Compatibility bridge for the original Rosmaster phone TCP protocol.

    The original rosmaster_main_ori.py opens the Rosmaster serial device itself.
    This node does not touch that device. It keeps the phone-facing TCP 6000
    protocol but translates motion commands into ROS2 /cmd_vel, so the Yahboom
    base driver remains the single hardware owner.
    """

    def __init__(self) -> None:
        super().__init__("jetcar_remote_bridge")
        self.declare_parameter("remote_control_host", "0.0.0.0")
        self.declare_parameter("remote_control_port", 6000)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("snapshot_topic", "/jetcar/snapshot")
        self.declare_parameter("max_linear_x", 0.35)
        self.declare_parameter("max_linear_y", 0.25)
        self.declare_parameter("max_angular_z", 0.8)
        self.declare_parameter("button_linear_x", 0.22)
        self.declare_parameter("button_linear_y", 0.18)
        self.declare_parameter("button_angular_z", 0.55)
        self.declare_parameter("command_timeout_seconds", 0.8)
        self.declare_parameter("battery_voltage_x10", 123)
        self.declare_parameter("firmware_version_x10", 35)

        self._cmd_pub = self.create_publisher(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            10,
        )
        self._snapshot_pub = self.create_publisher(
            Empty,
            str(self.get_parameter("snapshot_topic").value),
            10,
        )
        self._state = RemoteState()
        self._state_lock = threading.Lock()
        self._server = None
        self._server_thread = None

        self._start_server()
        self.create_timer(0.2, self._watchdog_tick)
        self.get_logger().info("JetCar remote bridge started")

    def destroy_node(self) -> bool:
        self._publish_stop()
        self._stop_server()
        return super().destroy_node()

    def _start_server(self) -> None:
        host = str(self.get_parameter("remote_control_host").value).strip()
        port = int(self.get_parameter("remote_control_port").value)
        if port <= 0:
            self.get_logger().info("remote control TCP server disabled")
            return
        node = self

        class Handler(socketserver.BaseRequestHandler):
            def handle(self) -> None:
                address = self.client_address[0] if self.client_address else ""
                node.get_logger().info(f"remote client connected: {self.client_address}")
                with node._state_lock:
                    node._state.client = address
                buffer = ""
                while True:
                    try:
                        raw = self.request.recv(1024)
                    except OSError:
                        break
                    if not raw:
                        break
                    buffer += raw.decode("utf-8", errors="replace")
                    while True:
                        start = buffer.rfind("$")
                        end = buffer.find("#", start + 1) if start >= 0 else -1
                        if start < 0 or end <= start:
                            if len(buffer) > 2048:
                                buffer = buffer[-256:]
                            break
                        packet = buffer[start : end + 1]
                        buffer = buffer[end + 1 :]
                        response = node._handle_packet(packet)
                        if response:
                            try:
                                self.request.sendall(response.encode("utf-8"))
                            except OSError:
                                return
                node._publish_stop()
                node.get_logger().info(f"remote client disconnected: {self.client_address}")

        class ThreadingServer(socketserver.ThreadingTCPServer):
            allow_reuse_address = True

        self._server = ThreadingServer((host, port), Handler)
        self._server_thread = threading.Thread(
            target=self._server.serve_forever,
            name="jetcar-remote-bridge",
            daemon=True,
        )
        self._server_thread.start()
        self.get_logger().info(f"remote control TCP server listening on {host}:{port}")

    def _stop_server(self) -> None:
        if self._server is None:
            return
        self._server.shutdown()
        self._server.server_close()
        if self._server_thread is not None:
            self._server_thread.join(timeout=2.0)
        self._server = None
        self._server_thread = None

    def _handle_packet(self, packet: str) -> str:
        try:
            parsed = _parse_packet(packet)
        except ValueError as exc:
            self.get_logger().warning(f"ignored bad remote packet: {exc}")
            return ""

        with self._state_lock:
            self._state.car_type = parsed["car_type"]
            self._state.last_command_at = time.monotonic()

        cmd = parsed["cmd"]
        payload = parsed["payload"]
        if cmd == "0F":
            return self._handle_enter_page(payload)
        if cmd == "01":
            return self._reply("01", [int(self.get_parameter("firmware_version_x10").value)])
        if cmd == "02":
            return self._reply("02", [int(self.get_parameter("battery_voltage_x10").value)])
        if cmd == "10":
            self._handle_joystick(payload)
            return ""
        if cmd == "15":
            self._handle_button(payload)
            return ""
        if cmd == "16":
            self._handle_speed(payload)
            return self._reply_speed()
        if cmd == "17":
            self._handle_stabilize(payload)
            return self._reply_stabilize()
        if cmd == "60":
            self._snapshot_pub.publish(Empty())
            return ""
        if cmd in {"61", "62"}:
            return ""
        self.get_logger().debug(f"remote command ignored cmd={cmd}")
        return ""

    def _handle_enter_page(self, payload: list[int]) -> str:
        page = payload[0] if payload else 0
        if page == 0:
            return self._reply("02", [int(self.get_parameter("battery_voltage_x10").value)])
        if page == 1:
            return self._reply_speed() + self._reply_stabilize()
        return ""

    def _handle_joystick(self, payload: list[int]) -> None:
        if len(payload) < 2:
            return
        x = _signed_u8(payload[0])
        y = _signed_u8(payload[1])
        linear_x = (y / 100.0) * float(self.get_parameter("max_linear_x").value)
        linear_y = (-x / 100.0) * float(self.get_parameter("max_linear_y").value)
        self._publish_twist(linear_x, linear_y, 0.0)

    def _handle_button(self, payload: list[int]) -> None:
        if not payload:
            return
        direction = payload[0]
        linear_x = float(self.get_parameter("button_linear_x").value)
        linear_y = float(self.get_parameter("button_linear_y").value)
        angular_z = float(self.get_parameter("button_angular_z").value)
        if direction == 0:
            self._publish_stop()
        elif direction == 1:
            self._publish_twist(linear_x, 0.0, 0.0)
        elif direction == 2:
            self._publish_twist(-linear_x, 0.0, 0.0)
        elif direction == 3:
            self._publish_twist(0.0, linear_y, 0.0)
        elif direction == 4:
            self._publish_twist(0.0, -linear_y, 0.0)
        elif direction == 5:
            self._publish_twist(0.0, 0.0, angular_z)
        elif direction == 6:
            self._publish_twist(0.0, 0.0, -angular_z)

    def _handle_speed(self, payload: list[int]) -> None:
        if len(payload) < 2:
            return
        with self._state_lock:
            self._state.speed_xy = max(0, min(100, payload[0]))
            self._state.speed_z = max(0, min(100, payload[1]))

    def _handle_stabilize(self, payload: list[int]) -> None:
        if not payload:
            return
        with self._state_lock:
            self._state.stabilize = 1 if payload[0] > 0 else 0

    def _reply_speed(self) -> str:
        with self._state_lock:
            speed_xy = self._state.speed_xy
            speed_z = self._state.speed_z
        return self._reply("16", [speed_xy, speed_z])

    def _reply_stabilize(self) -> str:
        with self._state_lock:
            stabilize = self._state.stabilize
        return self._reply("17", [stabilize])

    def _reply(self, cmd: str, payload: list[int]) -> str:
        with self._state_lock:
            car_type = self._state.car_type
        return _encode_packet(car_type, cmd, payload)

    def _publish_twist(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = float(linear_y)
        msg.angular.z = float(angular_z)
        self._cmd_pub.publish(msg)

    def _publish_stop(self) -> None:
        self._publish_twist(0.0, 0.0, 0.0)

    def _watchdog_tick(self) -> None:
        timeout = float(self.get_parameter("command_timeout_seconds").value)
        if timeout <= 0:
            return
        with self._state_lock:
            last = self._state.last_command_at
        if last > 0.0 and time.monotonic() - last > timeout:
            self._publish_stop()
            with self._state_lock:
                self._state.last_command_at = 0.0


def _parse_packet(packet: str) -> dict[str, Any]:
    text = packet.strip()
    if not text.startswith("$") or not text.endswith("#"):
        raise ValueError("missing packet delimiters")
    hex_text = text[1:-1]
    if len(hex_text) < 8 or len(hex_text) % 2 != 0:
        raise ValueError("invalid packet length")
    values = [int(hex_text[index : index + 2], 16) for index in range(0, len(hex_text), 2)]
    payload_len = values[2] - 2
    if payload_len < 0 or payload_len != len(values) - 4:
        raise ValueError("payload length mismatch")
    checksum = sum(values[:-1]) % 256
    if checksum != values[-1]:
        raise ValueError("checksum mismatch")
    return {
        "car_type": values[0],
        "cmd": f"{values[1]:02X}",
        "payload": values[3:-1],
    }


def _encode_packet(car_type: int, cmd: str, payload: list[int]) -> str:
    values = [
        int(car_type) & 0xFF,
        int(cmd, 16) & 0xFF,
        (len(payload) + 2) & 0xFF,
        *[int(item) & 0xFF for item in payload],
    ]
    values.append(sum(values) % 256)
    return "$" + "".join(f"{item:02x}" for item in values) + "#"


def _signed_u8(value: int) -> int:
    return int(value) - 256 if int(value) > 127 else int(value)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RemoteBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
