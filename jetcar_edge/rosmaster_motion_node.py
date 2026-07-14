from __future__ import annotations

import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

try:
    from Rosmaster_Lib import Rosmaster
except Exception:  # pragma: no cover - only available on Jetson runtime
    Rosmaster = None


class RosmasterMotionNode(Node):
    """Direct /cmd_vel -> Rosmaster bridge.

    The vendor phone app drives the chassis by calling Rosmaster_Lib directly.
    On the demo cars this path is more reliable than the full Yahboom bringup
    stack, so this node keeps the ROS /cmd_vel interface but sends the final
    command directly to the Rosmaster board.
    """

    def __init__(self) -> None:
        super().__init__("jetcar_rosmaster_motion")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("serial_port", "")
        self.declare_parameter("car_type", 1)
        self.declare_parameter("max_linear_x", 0.35)
        self.declare_parameter("max_linear_y", 0.25)
        self.declare_parameter("max_angular_z", 0.8)

        self._bot: Optional[Rosmaster] = None
        self._last_log_at = 0.0
        self._last_nonzero = False
        self._open_bot()
        self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self._on_cmd_vel,
            10,
        )
        self.get_logger().info(
            f"Rosmaster direct motion node listening on {self.get_parameter('cmd_vel_topic').value}"
        )

    def destroy_node(self) -> bool:
        self._stop()
        if self._bot is not None:
            try:
                del self._bot
            except Exception:
                pass
            self._bot = None
        return super().destroy_node()

    def _open_bot(self) -> None:
        if Rosmaster is None:
            raise RuntimeError("Rosmaster_Lib is not available in this environment")
        serial_port = str(self.get_parameter("serial_port").value).strip()
        if serial_port:
            self._bot = Rosmaster(com=serial_port)
            self.get_logger().info(f"opened Rosmaster serial port {serial_port}")
        else:
            self._bot = Rosmaster()
            self.get_logger().info("opened Rosmaster default serial port")
        self._bot.set_car_type(int(self.get_parameter("car_type").value))
        self._bot.create_receive_threading()

    def _on_cmd_vel(self, msg: Twist) -> None:
        if self._bot is None:
            return
        vx = _clamp(float(msg.linear.x), float(self.get_parameter("max_linear_x").value))
        vy = _clamp(float(msg.linear.y), float(self.get_parameter("max_linear_y").value))
        wz = _clamp(float(msg.angular.z), float(self.get_parameter("max_angular_z").value))
        self._log_motion(vx, vy, wz)
        self._bot.set_car_motion(vx, vy, wz)

    def _stop(self) -> None:
        if self._bot is None:
            return
        try:
            self._bot.set_car_motion(0.0, 0.0, 0.0)
            self._bot.set_motor(0, 0, 0, 0)
        except Exception as exc:
            self.get_logger().warning(f"failed to stop Rosmaster chassis: {exc}")

    def _log_motion(self, vx: float, vy: float, wz: float) -> None:
        nonzero = abs(vx) > 1e-4 or abs(vy) > 1e-4 or abs(wz) > 1e-4
        now = time.monotonic()
        if nonzero:
            if now - self._last_log_at >= 0.8:
                self.get_logger().info(f"cmd_vel -> Rosmaster vx={vx:.3f} vy={vy:.3f} wz={wz:.3f}")
                self._last_log_at = now
        elif self._last_nonzero:
            self.get_logger().info("cmd_vel -> Rosmaster stop")
        self._last_nonzero = nonzero


def _clamp(value: float, limit: float) -> float:
    limit = abs(limit)
    if limit <= 0:
        return 0.0
    return max(-limit, min(limit, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RosmasterMotionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
