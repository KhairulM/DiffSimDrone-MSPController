"""RealSense depth camera capture and preprocessing."""

import numpy as np
import pyrealsense2 as rs

import rospy
from sensor_msgs.msg import Image


class DepthCamera:
    """Handles RealSense depth camera capture and preprocessing for model input using Realsense SDK or ROS 1 topic."""

    def __init__(self, use_ros: bool = False, configs: dict = {}):
        """
        Initialize RealSense depth camera.

        Args:
            ros_configs: Dictionary with ROS 1 topic configurations
            rs_configs: Dictionary with RealSense configurations
                width: Depth stream width
                height: Depth stream height
                fps: Frames per second
        """
        self.use_ros = use_ros
        self.configs = configs

        # ROS 1 variables
        self.subscriber = None
        self.latest_depth_image = None
        self._rospy_initialized = False

        # RealSense variables
        self.pipeline = None
        self.profile = None
        self.depth_scale = None

        self.started = False

        self.started = False

    def start(self):
        """Start the RealSense depth streaming."""
        if self.started:
            print("Camera is already started.")
            return

        if self.use_ros:
            if not self._rospy_initialized:
                rospy.init_node("depth_camera_node", anonymous=True, disable_signals=True)
                self._rospy_initialized = True

            if self.subscriber is None:
                topic_name = self.configs.get(
                    "topic_name", "/camera_depth")
                queue_size = self.configs.get("queue_size", 10)
                self.subscriber = rospy.Subscriber(
                    topic_name,
                    Image,
                    self._ros_callback,
                    queue_size=queue_size
                )
                print(f"Subscribed to ROS 1 topic: {topic_name}")

            print("Using ROS 1 topic for depth images. No RealSense pipeline started.")
        else:
            self.pipeline = rs.pipeline()
            self.rs_config = rs.config()
            self.rs_config.enable_stream(
                rs.stream.depth, self.configs.get("width", 480), self.configs.get("height", 270), rs.format.z16, self.configs.get("fps", 30))

            self.profile = self.pipeline.start(self.rs_config)
            depth_sensor = self.profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            print(f"RealSense started. Depth scale: {self.depth_scale}")

        self.started = True

    def stop(self):
        """Stop the RealSense streaming."""
        if not self.started:
            print("Camera is not started.")
            return

        if self.use_ros:
            if self.subscriber is not None:
                self.subscriber.unregister()
                self.subscriber = None
            if self._rospy_initialized and not rospy.is_shutdown():
                rospy.signal_shutdown("DepthCamera stopped")
            self._rospy_initialized = False
        else:
            self.pipeline.stop()
            self.pipeline = None

        self.started = False

        print("Camera stopped.")

    def _ros_callback(self, msg: Image):
        """ROS 2 Image message callback to store the latest depth image."""
        if msg.encoding == "32FC1":
            depth_array = np.frombuffer(
                msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            self.latest_depth_image = depth_array
        elif msg.encoding == "16UC1":
            depth_array = np.frombuffer(
                msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            self.latest_depth_image = depth_array.astype(
                np.float32) * 0.001  # Convert mm to meters
        else:
            print(f"Unsupported encoding: {msg.encoding}")
            return

    def _resize_nearest(self, image: np.ndarray, target_h: int, target_w: int) -> np.ndarray:
        """Nearest-neighbor resize without extra dependencies."""
        h, w = image.shape
        y_idx = np.linspace(0, h - 1, target_h, dtype=np.int64)
        x_idx = np.linspace(0, w - 1, target_w, dtype=np.int64)
        return image[np.ix_(y_idx, x_idx)]

    def capture_and_preprocess(self) -> np.ndarray:
        """
        Capture depth frame from RealSense and preprocess for model input.

        Returns:
            Preprocessed depth image (1, 1, 12, 16) or None if capture failed
        """
        if not self.started:
            return None

        if self.use_ros:
            rospy.sleep(0.01)  # Allow callback thread to process messages
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
        # return np.zeros_like(pooled)[np.newaxis, np.newaxis, :, :].astype(np.float32)  # Placeholder for testing
