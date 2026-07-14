#!/usr/bin/env bash
set -euo pipefail

CONTAINER="${JETCAR_CONTAINER:-jetcar_auto}"
CLOUD_URL="${JETCAR_CLOUD_URL:-ws://192.168.175.90:8000/ws/video/car_001/camera_front/edge}"
WORKSPACE="${JETCAR_WORKSPACE:-/workspace/JetCarEdge}"
START_CAMERA="${JETCAR_START_CAMERA:-true}"

docker start "$CONTAINER" >/dev/null

docker exec "$CONTAINER" bash -lc "
set -e
pkill -f 'edge_bringup.launch.py' || true
pkill -f 'edge_upload_node' || true
pkill -f 'remote_bridge_node' || true
pkill -f 'task_orchestrator_node' || true
pkill -f 'Mcnamu_driver_X3' || true
pkill -f 'base_node_X3' || true
pkill -f 'imu_filter_madgwick' || true
pkill -f 'ekf_node' || true
pkill -f 'yahboom_joy_X3' || true
"

docker exec -d "$CONTAINER" bash -lc "
set +u
export ROS_DOMAIN_ID=30
source /opt/ros/foxy/setup.bash
source /root/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
source $WORKSPACE/install/setup.bash 2>/dev/null || true
set -u
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py
"

sleep 3

docker exec -d "$CONTAINER" bash -lc "
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
"

echo "JetCar services requested in container $CONTAINER"
echo "Expected ports after startup: 6000(remote), 6002(task), 8100(frame)"
