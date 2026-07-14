#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${JETCAR_WORKSPACE:-/workspace/JetCarEdge}"
CLOUD_URL="${JETCAR_CLOUD_URL:-ws://192.168.175.90:8000/ws/video/car_001/camera_front/edge}"
START_CAMERA="${JETCAR_START_CAMERA:-true}"
START_YAHBOOM_BASE="${JETCAR_START_YAHBOOM_BASE:-false}"
START_MOTION_DRIVER="${JETCAR_START_MOTION_DRIVER:-true}"
ROSMASTER_SERIAL_PORT="${JETCAR_ROSMASTER_SERIAL_PORT:-}"
LOG_DIR="${JETCAR_LOG_DIR:-/tmp/jetcar_edge_logs}"

mkdir -p "$LOG_DIR"
BASE_RUNNER="$LOG_DIR/run_base.sh"
EDGE_RUNNER="$LOG_DIR/run_edge.sh"

source_ros_env() {
  set +u
  source /opt/ros/foxy/setup.bash
  source /root/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
  source "$WORKSPACE/install/setup.bash"
  set -u
}

pkill -f 'edge_bringup.launch.py' || true
pkill -f 'edge_upload_node' || true
pkill -f 'remote_bridge_node' || true
pkill -f 'rosmaster_motion_node' || true
pkill -f 'task_orchestrator_node' || true
pkill -f 'Mcnamu_driver_X3' || true
pkill -f 'base_node_X3' || true
pkill -f 'imu_filter_madgwick' || true
pkill -f 'ekf_node' || true
pkill -f 'yahboom_joy_X3' || true

export ROS_DOMAIN_ID=30
source_ros_env

if [ "$START_YAHBOOM_BASE" = "true" ]; then
cat >"$BASE_RUNNER" <<EOF
#!/usr/bin/env bash
set -eo pipefail
echo "[\$(date -Is)] starting Yahboom base bringup"
set +u
export ROS_DOMAIN_ID=30
source /opt/ros/foxy/setup.bash
source /root/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
source $WORKSPACE/install/setup.bash
set -u
echo "[\$(date -Is)] ROS env loaded for base"
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py
EOF
chmod +x "$BASE_RUNNER"
nohup "$BASE_RUNNER" >"$LOG_DIR/base.log" 2>&1 &
BASE_PID=$!
echo "$BASE_PID" >"$LOG_DIR/base.pid"
else
  echo "[$(date -Is)] Yahboom base bringup disabled; using Edge direct Rosmaster motion driver" >"$LOG_DIR/base.log"
fi

sleep 1

cat >"$EDGE_RUNNER" <<EOF
#!/usr/bin/env bash
set -eo pipefail
echo "[\$(date -Is)] starting JetCarEdge"
set +u
export ROS_DOMAIN_ID=30
source /opt/ros/foxy/setup.bash
source /root/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
cd $WORKSPACE
source install/setup.bash
set -u
echo "[\$(date -Is)] ROS env loaded for edge"
echo "[\$(date -Is)] cloud_url=$CLOUD_URL start_camera=$START_CAMERA start_motion_driver=$START_MOTION_DRIVER"
ARGS=(
  "cloud_url:=$CLOUD_URL"
  "start_base:=false"
  "start_camera:=$START_CAMERA"
  "start_motion_driver:=$START_MOTION_DRIVER"
  "start_remote_bridge:=true"
  "start_task_orchestrator:=true"
)
if [ -n "$ROSMASTER_SERIAL_PORT" ]; then
  ARGS+=("rosmaster_serial_port:=$ROSMASTER_SERIAL_PORT")
fi
ros2 launch jetcar_edge edge_bringup.launch.py "\${ARGS[@]}"
EOF
chmod +x "$EDGE_RUNNER"
nohup "$EDGE_RUNNER" >"$LOG_DIR/edge.log" 2>&1 &
EDGE_PID=$!
echo "$EDGE_PID" >"$LOG_DIR/edge.pid"

sleep 2
if ! ps -p "$EDGE_PID" >/dev/null 2>&1; then
  echo "JetCarEdge failed to stay running. See $LOG_DIR/edge.log" >&2
  tail -n 80 "$LOG_DIR/edge.log" >&2 || true
  exit 1
fi

echo "JetCar services started."
echo "Logs:"
echo "  $LOG_DIR/base.log"
echo "  $LOG_DIR/edge.log"
echo "Expected ports: 6000, 6002, 8100"
ps -ef | grep -E 'edge_bringup|edge_upload|remote_bridge|rosmaster_motion|task_orchestrator|yahboomcar_bringup_X3' | grep -v grep || true
