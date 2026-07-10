# JetCarEdge

JetCarEdge is the Jetson-side data bridge for the JetCar project. It runs as a
ROS2 Python package, subscribes to camera/lidar/IMU topics, compresses camera
frames, uploads them to the cloud inference service by WebSocket, and publishes
AI results back into ROS2 for local safety handling.

## Technology Choice

- Runtime: Ubuntu on Jetson with ROS2 Foxy or Humble.
- Language: Python 3, because ROS2 Python nodes are fast to iterate and match
  the image/sensor upload workflow.
- ROS2 libraries: `rclpy`, `sensor_msgs`, `std_msgs`, `cv_bridge`.
- Network: `websocket-client`, using a persistent WebSocket connection to the
  cloud service.
- Image processing: OpenCV JPEG resize/compression before upload.

This repository intentionally does not replace the car's existing TCP remote
control service. The Flutter app can keep controlling the car over TCP while
this node only handles camera/sensor upload and AI result feedback.

## Repository Layout

```text
JetCarEdge/
  jetcar_edge/
    edge_upload_node.py     ROS2 node entrypoint
    image_codec.py          ROS Image -> JPEG base64 conversion
    models.py               Message schema helpers
    safety.py               Local danger decision helper
    sensor_buffer.py        Latest lidar/IMU cache
    ws_client.py            Reconnecting WebSocket worker
  config/
    edge.example.yaml       Example runtime configuration
  resource/
    jetcar_edge             ROS2 ament marker
  package.xml               ROS2 package metadata
  setup.py                  ROS2 Python package setup
  requirements.txt          Python-only dependencies
```

## Environment Commands To Run

Run these on the Jetson after copying this folder into a ROS2 workspace, for
example `~/yahboomcar_ws/src/JetCarEdge`.

```bash
cd ~/yahboomcar_ws/src
cp -r /path/to/JetCarEdge .

python3 -m pip install -r JetCarEdge/requirements.txt

cd ~/yahboomcar_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select jetcar_edge
source install/setup.bash
```

## Mock Camera Upload

When the real car/camera is not available, upload one local image as the cloud
reference frame:

```bash
cd /path/to/JetCarEdge
python scripts/upload_mock_camera.py \
  --cloud http://192.168.137.1:8000 \
  --car-id car_001 \
  --image ../yolov5-7.0/data/images/bus.jpg
```

After this succeeds, the mobile app can upload another image to compare against
this simulated camera frame.

## Mock Camera Server

For the newer request-response flow, run a tiny HTTP server on the edge side.
The cloud will request the current frame only when the phone uploads a query
image:

```bash
cd /path/to/JetCarEdge
python scripts/mock_camera_server.py \
  --host 0.0.0.0 \
  --port 8100 \
  --image ../yolov5-7.0/data/images/bus.jpg
```

Then configure JetCarCloud:

```bash
EDGE_FRAME_URL=http://127.0.0.1:8100/api/frame
```

Later, this server can keep the same `/api/frame` interface and replace the
fixed file read with a camera/video-frame capture.

Start the node after the cloud service is already listening:

```bash
ros2 run jetcar_edge edge_upload_node \
  --ros-args \
  -p car_id:=car_001 \
  -p cloud_url:=ws://192.168.137.1:8000/ws/inference/car_001/edge \
  -p camera_topic:=/camera/image_raw \
  -p scan_topic:=/scan \
  -p imu_topic:=/imu/data
```

Useful control topics:

```bash
ros2 topic pub /jetcar/ai_enable std_msgs/msg/Bool "{data: true}" --once
ros2 topic pub /jetcar/snapshot std_msgs/msg/Empty "{}" --once
ros2 topic echo /jetcar/ai_result
ros2 topic echo /jetcar/emergency_stop
```

## Message Contract

The edge node sends:

```json
{
  "type": "edge_frame",
  "car_id": "car_001",
  "timestamp": 1720000000.12,
  "image": {
    "encoding": "jpeg",
    "width": 640,
    "height": 480,
    "data": "base64-jpeg"
  },
  "sensors": {
    "lidar": {
      "angle_min": -3.14,
      "angle_increment": 0.01,
      "ranges": [1.2, 1.3]
    },
    "imu": {
      "orientation": [0, 0, 0, 1],
      "angular_velocity": [0, 0, 0],
      "linear_acceleration": [0, 0, 0]
    }
  }
}
```

The cloud service returns:

```json
{
  "type": "yolo_fusion",
  "car_id": "car_001",
  "edge_timestamp": 1720000000.12,
  "server_latency_ms": 18.5,
  "detections": [
    {
      "label": "person",
      "confidence": 0.91,
      "bbox": [120, 80, 320, 420],
      "distance_m": 2.4
    }
  ]
}
```
