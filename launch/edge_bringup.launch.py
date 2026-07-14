from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    car_id = LaunchConfiguration("car_id")
    stream_id = LaunchConfiguration("stream_id")
    cloud_url = LaunchConfiguration("cloud_url")
    camera_topic = LaunchConfiguration("camera_topic")
    start_base = LaunchConfiguration("start_base")
    start_camera = LaunchConfiguration("start_camera")
    start_motion_driver = LaunchConfiguration("start_motion_driver")
    rosmaster_serial_port = LaunchConfiguration("rosmaster_serial_port")
    start_remote_bridge = LaunchConfiguration("start_remote_bridge")
    remote_control_port = LaunchConfiguration("remote_control_port")
    app_control_port = LaunchConfiguration("app_control_port")
    frame_server_port = LaunchConfiguration("frame_server_port")
    task_control_port = LaunchConfiguration("task_control_port")
    start_task_orchestrator = LaunchConfiguration("start_task_orchestrator")
    upload_fps = LaunchConfiguration("upload_fps")
    image_width = LaunchConfiguration("image_width")
    jpeg_quality = LaunchConfiguration("jpeg_quality")

    base_driver = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "yahboomcar_bringup",
            "Mcnamu_driver_X3",
        ],
        condition=IfCondition(start_base),
        output="screen",
    )

    camera_launch = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "astra_camera",
            "astro_pro_plus.launch.xml",
            "enable_color:=true",
            "enable_depth:=false",
        ],
        condition=IfCondition(start_camera),
        output="screen",
    )

    edge_node = Node(
        package="jetcar_edge",
        executable="edge_upload_node",
        name="jetcar_edge_upload",
        output="screen",
        parameters=[
            {
                "car_id": car_id,
                "stream_id": stream_id,
                "cloud_url": cloud_url,
                "camera_topic": camera_topic,
                "algorithm_ids": "",
                "app_control_port": app_control_port,
                "frame_server_port": frame_server_port,
                "upload_fps": upload_fps,
                "image_width": image_width,
                "jpeg_quality": jpeg_quality,
                "docker_orchestrator_enabled": False,
            }
        ],
    )

    motion_node = Node(
        package="jetcar_edge",
        executable="rosmaster_motion_node",
        name="jetcar_rosmaster_motion",
        output="screen",
        condition=IfCondition(start_motion_driver),
        parameters=[
            {
                "cmd_vel_topic": "/cmd_vel",
                "serial_port": rosmaster_serial_port,
                "car_type": 1,
            }
        ],
    )

    remote_bridge_node = Node(
        package="jetcar_edge",
        executable="remote_bridge_node",
        name="jetcar_remote_bridge",
        output="screen",
        condition=IfCondition(start_remote_bridge),
        parameters=[
            {
                "remote_control_port": remote_control_port,
                "cmd_vel_topic": "/cmd_vel",
                "snapshot_topic": "/jetcar/snapshot",
                "algorithm_control_topic": "/jetcar/algorithm_ids",
            }
        ],
    )

    task_node = Node(
        package="jetcar_edge",
        executable="task_orchestrator_node",
        name="jetcar_edge_tasks",
        output="screen",
        condition=IfCondition(start_task_orchestrator),
        parameters=[
            {
                "task_control_port": task_control_port,
                "algorithm_control_topic": "/jetcar/algorithm_ids",
                "ai_result_topic": "/jetcar/ai_result",
                "task_status_topic": "/jetcar/task_status",
                "cmd_vel_topic": "/cmd_vel",
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("car_id", default_value="car_001"),
            DeclareLaunchArgument("stream_id", default_value="camera_front"),
            DeclareLaunchArgument(
                "cloud_url",
                default_value="ws://192.168.175.90:8000/ws/video/car_001/camera_front/edge",
            ),
            DeclareLaunchArgument("camera_topic", default_value="/camera/color/image_raw"),
            DeclareLaunchArgument("start_base", default_value="true"),
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("start_motion_driver", default_value="true"),
            DeclareLaunchArgument("rosmaster_serial_port", default_value=""),
            DeclareLaunchArgument("start_remote_bridge", default_value="true"),
            DeclareLaunchArgument("remote_control_port", default_value="6000"),
            DeclareLaunchArgument("app_control_port", default_value="0"),
            DeclareLaunchArgument("frame_server_port", default_value="8100"),
            DeclareLaunchArgument("task_control_port", default_value="6002"),
            DeclareLaunchArgument("start_task_orchestrator", default_value="true"),
            DeclareLaunchArgument("upload_fps", default_value="3.0"),
            DeclareLaunchArgument("image_width", default_value="512"),
            DeclareLaunchArgument("jpeg_quality", default_value="60"),
            base_driver,
            camera_launch,
            motion_node,
            edge_node,
            remote_bridge_node,
            task_node,
        ]
    )
