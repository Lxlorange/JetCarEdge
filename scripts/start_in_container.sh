#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${JETCAR_WORKSPACE:-/workspace/JetCarEdge}"
CLOUD_URL="${JETCAR_CLOUD_URL:-ws://192.168.175.90:8000/ws/video/car_001/camera_front/edge}"
START_CAMERA="${JETCAR_START_CAMERA:-true}"
LOG_DIR="${JETCAR_LOG_DIR:-/tmp/jetcar_edge_logs}"

mkdir -p "$LOG_DIR"

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
pkill -f 'task_orchestrator_node' || true
pkill -f 'Mcnamu_driver_X3' || true
pkill -f 'base_node_X3' || true
pkill -f 'imu_filter_madgwick' || true
pkill -f 'ekf_node' || true
pkill -f 'yahboom_joy_X3' || true

export ROS_DOMAIN_ID=30
source_ros_env

nohup bash -lc "
set +u
export ROS_DOMAIN_ID=30
source /opt/ros/foxy/setup.bash
source /root/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
source $WORKSPACE/install/setup.bash
set -u
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py
" >"$LOG_DIR/base.log" 2>&1 &

sleep 3

nohup bash -lc "
set +u
export ROS_DOMAIN_ID=30
source /opt/ros/foxy/setup.bash
source /root/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
cd $WORKSPACE
source install/setup.bash
set -u
ros2 launch jetcar_edge edge_bringup.launch.py \
  cloud_url:=$CLOUD_URL \
  start_base:=false \
  start_camera:=$START_CAMERA \
  start_remote_bridge:=true \
  start_task_orchestrator:=true
" >"$LOG_DIR/edge.log" 2>&1 &

echo "JetCar services started."
echo "Logs:"
echo "  $LOG_DIR/base.log"
echo "  $LOG_DIR/edge.log"
echo "Expected ports: 6000, 6002, 8100"
