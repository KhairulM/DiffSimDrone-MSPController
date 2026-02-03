"""RealSense depth camera capture and preprocessing."""

import numpy as np
import pyrealsense2 as rs


class DepthCamera:
    """Handles RealSense depth camera capture and preprocessing for model input."""

    def __init__(self, width: int = 256, height: int = 144, fps: int = 90):
        """
        Initialize RealSense depth camera.

        Args:
            width: Depth stream width
            height: Depth stream height
            fps: Frames per second
        """
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(
            rs.stream.depth, width, height, rs.format.z16, fps)
        self.profile = None
        self.depth_scale = None
        self.started = False

    def start(self):
        """Start the RealSense depth streaming pipeline."""
        if self.started:
            return
        self.profile = self.pipeline.start(self.config)
        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        self.started = True
        print(f"RealSense started. Depth scale: {self.depth_scale}")

    def stop(self):
        """Stop the RealSense pipeline."""
        if self.started:
            self.pipeline.stop()
            self.started = False
            print("RealSense stopped.")

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
