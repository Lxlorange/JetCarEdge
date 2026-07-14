#!/usr/bin/env bash
set -euo pipefail

pkill -f 'edge_bringup.launch.py' || true
pkill -f 'edge_upload_node' || true
pkill -f 'remote_bridge_node' || true
pkill -f 'rosmaster_motion_node' || true
pkill -f 'task_orchestrator_node' || true
pkill -f 'astra_camera' || true
pkill -f 'Mcnamu_driver_X3' || true
pkill -f 'base_node_X3' || true
pkill -f 'imu_filter_madgwick' || true
pkill -f 'ekf_node' || true
pkill -f 'yahboom_joy_X3' || true

echo "JetCar services stopped."
