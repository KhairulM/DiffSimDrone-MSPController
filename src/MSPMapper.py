from .attitude_controller import AttitudeController
from .model_inference import NavigationModel
from .depth_camera import DepthCamera
import numpy as np
import messages as msg
from Copter import Copter
import sys
from pathlib import Path

# Add the Control-Betaflight-Copter src directory to the path
_lib_path = Path(__file__).parent.parent / "lib" / \
    "Control-Betaflight-Copter" / "src"
sys.path.insert(0, str(_lib_path))


class MSPMapper(Copter):
    def __init__(self, stop_cmd=None, model_path=None):
        super().__init__(stop_cmd)

        # Initialize submodules
        self.depth_camera = DepthCamera(width=256, height=144, fps=90)
        self.model = NavigationModel(model_path)
        self.attitude_controller = AttitudeController(
            rc_center=1500,
            rc_range=500,
            max_angle_deg=30.0,
            max_yaw_rate=180.0
        )

        # Copter state tracking
        self.copter_data["copter_state"] = None
        self.copter_data["g_std"] = np.array(
            [0, 0, -9.80665], dtype=np.float32)  # Gravity vector in m/s^2

        # Navigation state
        self.copter_data["velocity"] = np.zeros(
            3, dtype=np.float32)  # Estimated velocity (world frame)
        self.copter_data["position"] = np.zeros(
            3, dtype=np.float32)  # Estimated position
        self.copter_data["target_position"] = np.array(
            [5.0, 0.0, 1.5], dtype=np.float32)  # Target position
        # Obstacle clearance margin (from depth)
        self.copter_data["margin"] = np.float32(1.0)
        self.copter_data["max_speed"] = np.float32(3.0)  # Maximum speed m/s

    def start_realsense(self):
        """Start the RealSense depth streaming pipeline."""
        self.depth_camera.start()

    def stop_realsense(self):
        """Stop the RealSense pipeline."""
        self.depth_camera.stop()

    def reset_hidden_state(self):
        """Reset the recurrent hidden state"""
        self.model.reset_hidden_state()

    def set_target_position(self, target: np.ndarray):
        """Set the navigation target position"""
        self.copter_data["target_position"] = target.astype(np.float32)

    def update_velocity(self, velocity: np.ndarray):
        """Update velocity estimate (world frame)"""
        self.copter_data["velocity"] = velocity.astype(np.float32)

    def update_position(self, position: np.ndarray):
        """Update position estimate"""
        self.copter_data["position"] = position.astype(np.float32)

    def update_margin(self, margin: float):
        """Update obstacle clearance margin (from depth processing)"""
        self.copter_data["margin"] = np.float32(margin)

    def _get_attitude_from_telemetry(self) -> tuple[float, float, float]:
        """
        Get roll, pitch, yaw from telemetry data.
        Returns angles in radians.
        """
        attitude = self.copter_data['attitude']
        # Telemetry provides angles in degrees
        roll = np.radians(
            attitude['angx']) if attitude['angx'] is not None else 0.0
        pitch = np.radians(
            attitude['angy']) if attitude['angy'] is not None else 0.0
        yaw = np.radians(attitude['heading']
                         ) if attitude['heading'] is not None else 0.0
        return roll, pitch, yaw

    def _compute_rotation_matrix(self) -> np.ndarray:
        """
        Compute rotation matrix from telemetry attitude (roll, pitch, yaw).
        Uses ZYX Euler angle convention.
        """
        roll, pitch, yaw = self._get_attitude_from_telemetry()

        # Rotation matrices for each axis
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)

        # ZYX Euler rotation matrix
        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr]
        ], dtype=np.float32)

        return R

    def _compute_local_frame_rotation(self) -> np.ndarray:
        """
        Compute the local frame rotation matrix (yaw-only, level frame).
        This aligns with the ground plane but follows drone's heading.
        Uses heading from telemetry.
        """
        _, _, yaw = self._get_attitude_from_telemetry()

        # Simple yaw-only rotation (level frame)
        cy, sy = np.cos(yaw), np.sin(yaw)
        R_local = np.array([
            [cy, -sy, 0],
            [sy,  cy, 0],
            [0,   0,  1]
        ], dtype=np.float32)

        return R_local

    def _build_observation(self) -> np.ndarray:
        """
        Build the observation vector for the model.
        observation = [local_v (3), target_v_local (3), up_vector (3), margin (1)]
        """
        R = self._compute_rotation_matrix()  # Get R from telemetry attitude
        R_local = self._compute_local_frame_rotation()
        velocity = self.copter_data["velocity"]
        position = self.copter_data["position"]
        target = self.copter_data["target_position"]
        max_speed = self.copter_data["max_speed"]
        margin = self.copter_data["margin"]

        # Compute target velocity (direction to target, clamped to max_speed)
        target_v_raw = target - position
        target_v_norm = np.linalg.norm(target_v_raw)
        if target_v_norm > 1e-6:
            target_v_unit = target_v_raw / target_v_norm
            target_v = target_v_unit * min(target_v_norm, max_speed)
        else:
            target_v = np.zeros(3, dtype=np.float32)

        # Transform velocity to local frame
        local_v = R_local.T @ velocity  # (3,)

        # Transform target velocity to local frame
        target_v_local = R_local.T @ target_v  # (3,)

        # Up vector from rotation matrix (z-axis of body frame)
        up_vector = R[:, 2]  # (3,)

        # Build observation: [local_v, target_v_local, up_vector, margin]
        observation = np.concatenate([
            local_v,
            target_v_local,
            up_vector,
            [margin]
        ]).astype(np.float32)

        return observation[np.newaxis, :]  # (1, 10)

    def update_copter_state(self):
        if self.copter_data['aux3'] is None:
            msg.display(msg.copter_msp_not_ready)
            return

        # Enable the control iteration only in AUTO mode
        if self.copter_data['aux3'] >= 1600:
            self.copter_data['copter_state'] = 'AUTO'
        elif self.copter_data['aux3'] <= 1400:
            self.copter_data['copter_state'] = 'FAILSAFE'
        else:
            self.copter_data['copter_state'] = 'REMOTE'

    def control_iteration(self):
        self.update_copter_state()

        # Only control in 'AUTO' state
        if self.copter_data['copter_state'] != 'AUTO':
            self.set_rc(self.default_control_rates | self.default_aux_values)
            self.reset_hidden_state()  # Reset RNN state when not in AUTO
            return

        # Capture and preprocess depth from RealSense
        image = self.depth_camera.capture_and_preprocess()  # (1, 1, 12, 16)
        if image is None:
            msg.display("Depth frame not available")
            self.set_rc(self.default_control_rates | self.default_aux_values)
            return

        # 1. Build observation vector
        observation = self._build_observation()  # (1, 10)

        # 2. Run model inference
        action = self.model.run(image, observation)  # (1, 6)

        # 3. Convert action to desired acceleration
        R_local = self._compute_local_frame_rotation()
        acc = self.model.action_to_acceleration(
            action,
            R_local,
            self.copter_data["g_std"]
        )

        # 4. Convert acceleration to RC commands
        roll, pitch, yaw = self._get_attitude_from_telemetry()
        rc_commands = self.attitude_controller.acceleration_to_rc(
            acc,
            yaw,  # Current yaw from telemetry
            self.copter_data["position"],
            self.copter_data["target_position"],
            self.copter_data["g_std"]
        )

        # 5. Send RC commands
        self.set_rc(rc_commands | self.default_aux_values)
