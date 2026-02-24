"""
ROS 2 launch file that starts:
  1. The MAVROS node (connects to the FCU via serial / UDP).
  2. The MavrosMapper navigation controller node.

Usage
-----
    ros2 launch launch/mavros_mapper.launch.py

Override defaults with launch arguments:
    ros2 launch launch/mavros_mapper.launch.py \
        fcu_url:=udp://:14540@127.0.0.1:14557 \
        model_path:=/path/to/model.onnx \
        config:=configs/mavros.yaml \
        control_freq:=30.0
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---- Workspace root (two levels up from this file) --------------------
    ws_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    # ---- Launch arguments -------------------------------------------------
    fcu_url_arg = DeclareLaunchArgument(
        "fcu_url",
        default_value="/dev/ttyACM0:921600",
        description="FCU connection URL (serial or UDP). "
                    "Examples: /dev/ttyACM0:921600, udp://:14540@127.0.0.1:14557",
    )
    gcs_url_arg = DeclareLaunchArgument(
        "gcs_url",
        default_value="",
        description="GCS bridge URL (leave empty to disable).",
    )
    tgt_system_arg = DeclareLaunchArgument(
        "tgt_system_id",
        default_value="1",
        description="MAVLink target system ID.",
    )
    tgt_component_arg = DeclareLaunchArgument(
        "tgt_component_id",
        default_value="1",
        description="MAVLink target component ID.",
    )
    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value=os.path.join(ws_root, "base.onnx"),
        description="Path to the ONNX navigation model.",
    )
    config_arg = DeclareLaunchArgument(
        "config",
        default_value=os.path.join(ws_root, "configs", "mavros.yaml"),
        description="Path to MavrosMapper YAML config.",
    )
    control_freq_arg = DeclareLaunchArgument(
        "control_freq",
        default_value="20.0",
        description="Control loop frequency (Hz).",
    )
    depth_ros_arg = DeclareLaunchArgument(
        "depth_ros",
        default_value="false",
        description="Use ROS topic for depth (true) or native RealSense SDK (false).",
    )

    # ---- MAVROS node ------------------------------------------------------
    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        name="mavros",
        output="screen",
        parameters=[{
            "fcu_url": LaunchConfiguration("fcu_url"),
            "gcs_url": LaunchConfiguration("gcs_url"),
            "tgt_system_id": LaunchConfiguration("tgt_system_id"),
            "tgt_component_id": LaunchConfiguration("tgt_component_id"),
            "fcu_protocol": "v2.0",
        }],
    )

    # ---- MavrosMapper node (as a ROS 2 process) --------------------------
    # We run the Python script directly because the project is not a colcon
    # package.  `ros2 run` is not needed when we specify the executable.
    mapper_node = Node(
        package=None,  # standalone executable
        executable="python3",
        name="mavros_mapper",
        output="screen",
        arguments=[
            os.path.join(ws_root, "src", "MavrosMapper.py"),
            "--model_path", LaunchConfiguration("model_path"),
            "--config", LaunchConfiguration("config"),
            "--control_freq", LaunchConfiguration("control_freq"),
        ],
        # Additional env so Python can find project modules
        additional_env={
            "PYTHONPATH": os.path.join(ws_root, "src")
                          + ":" + os.environ.get("PYTHONPATH", ""),
        },
    )

    return LaunchDescription([
        fcu_url_arg,
        gcs_url_arg,
        tgt_system_arg,
        tgt_component_arg,
        model_path_arg,
        config_arg,
        control_freq_arg,
        depth_ros_arg,
        LogInfo(msg="Launching MAVROS + MavrosMapper"),
        mavros_node,
        mapper_node,
    ])
