"""RealSense depth camera capture and preprocessing."""

import numpy as np
import pyrealsense2 as rs

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class DepthCamera:
    """Handles RealSense depth camera capture and preprocessing for model input using Realsense SDK or ROS 2 topic."""

    def __init__(self, ros_configs=None, rs_configs=None):
        """
        Initialize RealSense depth camera.

        Args:
            ros_configs: Dictionary with ROS 2 topic configurations
            rs_configs: Dictionary with RealSense configurations
                width: Depth stream width
                height: Depth stream height
                fps: Frames per second
        """
        if ros_configs not in (None, {}):
            self.use_ros = True
            self.ros_configs = ros_configs
            self.node = None
            self.subscription = None
            self.latest_depth_image = None
            self._rclpy_initialized = False
        else:
            self.use_ros = False
            self.ros_configs = {}
            self._rclpy_initialized = False
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            rs_configs = rs_configs or {"width": 256, "height": 144, "fps": 90}
            self.config.enable_stream(
                rs.stream.depth, rs_configs["width"], rs_configs["height"], rs.format.z16, rs_configs["fps"])
            self.profile = None
            self.depth_scale = None

        self.started = False

    def start(self):
        """Start the RealSense depth streaming."""
        if self.started:
            return

        if self.use_ros:
            if not rclpy.ok():
                rclpy.init()
                self._rclpy_initialized = True

            if self.node is None:
                self.node = Node("depth_camera_node")
                topic_name = self.ros_configs.get(
                    "topic_name", "/camera_depth")
                self.subscription = self.node.create_subscription(
                    Image,
                    topic_name,
                    self._ros_callback,
                    rclpy.qos.QoSProfile(
                        depth=10, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE),
                )
                print(f"Subscribed to ROS 2 topic: {topic_name}")

            print("Using ROS 2 topic for depth images. No RealSense pipeline started.")
            self.started = True
        else:
            self.profile = self.pipeline.start(self.config)
            depth_sensor = self.profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            print(f"RealSense started. Depth scale: {self.depth_scale}")

            self.started = True

    def stop(self):
        """Stop the RealSense streaming."""
        if self.started:
            if self.use_ros:
                if self.node is not None:
                    self.node.destroy_node()
                    self.node = None
                if self._rclpy_initialized and rclpy.ok():
                    rclpy.shutdown()
                self._rclpy_initialized = False
            else:
                self.pipeline.stop()

            self.started = False
            print("RealSense stopped.")

    def _ros_callback(self, msg: Image):
        """ROS 2 Image message callback to store the latest depth image."""
        if msg.encoding != "16UC1":
            print(f"Unsupported encoding: {msg.encoding}")
            return
        depth_array = np.frombuffer(
            msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        self.latest_depth_image = depth_array.astype(
            np.float32) * 0.001  # Convert mm to meters

    def _resize_nearest(self, image: np.ndarray, target_h: int, target_w: int) -> np.ndarray:
        """Nearest-neighbor resize without extra dependencies."""
        h, w = image.shape
        y_idx = np.linspace(0, h - 1, target_h, dtype=np.int64)
        x_idx = np.linspace(0, w - 1, target_w, dtype=np.int64)
        return image[np.ix_(y_idx, x_idx)]

    def capture_and_preprocess(self) -> np.ndarray | None:
        """
        Capture depth frame from RealSense and preprocess for model input.

        Returns:
            Preprocessed depth image (1, 1, 12, 16) or None if capture failed
        """
        if not self.started:
            return None

        if self.use_ros:
            rclpy.spin_once(self.node, timeout_sec=1.0)
            if self.latest_depth_image is None:
                return None
            depth_m = self.latest_depth_image
        else:
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                return None

            # Get depth data in meters
            depth_m = np.asanyarray(depth_frame.get_data()).astype(np.float32)
            depth_m *= self.depth_scale

        # Crop to 4:3 aspect ratio (center crop)
        h, w = depth_m.shape
        target_w = int(h * 4 / 3)
        if target_w < w:
            start = (w - target_w) // 2
            depth_m = depth_m[:, start:start + target_w]

        # Compute inverse depth
        valid = (depth_m > 0.3) & (depth_m < 24)
        inv_depth = np.zeros_like(depth_m)
        np.divide(3.0, depth_m, out=inv_depth, where=valid)

        # Normalize inverse depth (zero mean, unit std for valid pixels)
        if valid.any():
            valid_values = inv_depth[valid]
            mean = valid_values.mean()
            std = valid_values.std()
            if std > 1e-6:
                inv_depth[valid] = (valid_values - mean) / std

        # Resize to 24x32 using nearest neighbor
        resized = self._resize_nearest(inv_depth, 24, 32)

        # 2x2 max pool to get (12, 16) - keeps nearest point per grid cell
        pooled = resized.reshape(12, 2, 16, 2).max(axis=(1, 3))

        return pooled[np.newaxis, np.newaxis, :, :].astype(np.float32)
