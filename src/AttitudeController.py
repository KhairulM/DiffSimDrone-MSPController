"""Attitude controller for converting acceleration to RC commands."""

import numpy as np


class AttitudeController:
    """Converts desired acceleration to RC commands (roll, pitch, yaw, throttle)."""

    def __init__(
        self,
        rc_center: int = 1500,
        rc_range: int = 500,
        max_angle_deg: float = 30.0,
        max_yaw_rate: float = 180.0,
        g_mag: float = 9.80665
    ):
        """
        Initialize attitude controller.

        Args:
            rc_center: Center RC value (neutral)
            rc_range: RC range from center (1000-2000 gives ±500)
            max_angle_deg: Maximum roll/pitch angle in degrees
            max_yaw_rate: Maximum yaw rate in degrees/second
            g_mag: Gravity magnitude in m/s^2
        """
        self.rc_center = rc_center
        self.rc_range = rc_range
        self.max_angle_deg = max_angle_deg
        self.max_yaw_rate = max_yaw_rate
        self.g_mag = g_mag

    def acceleration_to_rc(
        self,
        acc: np.ndarray,
        current_yaw: float,
        position: np.ndarray,
        target_position: np.ndarray,
        g_std: np.ndarray
    ) -> dict:
        """
        Convert desired acceleration to RC commands.

        Args:
            acc: Desired acceleration in world frame (3,)
            current_yaw: Current yaw angle in radians (from telemetry)
            position: Current position (3,)
            target_position: Target position (3,)
            g_std: Gravity vector (3,)

        Returns:
            Dictionary with 'roll', 'pitch', 'yaw', 'throttle' RC values
        """
        # Desired thrust vector (acceleration + gravity compensation)
        thrust_vec = acc - g_std

        # Thrust magnitude (normalized)
        thrust_mag = np.linalg.norm(thrust_vec)
        if thrust_mag < 1e-6:
            thrust_mag = self.g_mag
            thrust_vec = np.array([0, 0, self.g_mag], dtype=np.float32)

        # Normalize thrust vector to get desired body z-axis
        z_des = thrust_vec / thrust_mag

        # Compute desired roll and pitch from thrust vector
        # Roll: rotation about x-axis, affects y-component
        # Pitch: rotation about y-axis, affects x-component
        desired_pitch = np.arcsin(np.clip(-z_des[0], -1, 1))  # Forward tilt
        cos_pitch = np.cos(desired_pitch)
        if abs(cos_pitch) > 1e-6:
            desired_roll = np.arcsin(
                np.clip(z_des[1] / cos_pitch, -1, 1))  # Side tilt
        else:
            desired_roll = 0.0

        # Yaw: point towards target (optional, can be disabled)
        to_target = target_position - position
        to_target[2] = 0  # Project to horizontal
        if np.linalg.norm(to_target) > 0.5:  # Only adjust yaw if far enough
            desired_yaw = np.arctan2(to_target[1], to_target[0])
            yaw_error = desired_yaw - current_yaw
            # Normalize to [-pi, pi]
            yaw_error = np.arctan2(np.sin(yaw_error), np.cos(yaw_error))
        else:
            yaw_error = 0.0

        # Convert to RC values
        roll_deg = np.degrees(desired_roll)
        pitch_deg = np.degrees(desired_pitch)
        yaw_rate_deg = np.degrees(yaw_error) * 2.0  # P-gain for yaw rate

        # Clamp angles
        roll_deg = np.clip(roll_deg, -self.max_angle_deg, self.max_angle_deg)
        pitch_deg = np.clip(pitch_deg, -self.max_angle_deg, self.max_angle_deg)
        yaw_rate_deg = np.clip(
            yaw_rate_deg, -self.max_yaw_rate, self.max_yaw_rate)

        # Map to RC values (1000-2000)
        roll_rc = int(self.rc_center + (roll_deg /
                      self.max_angle_deg) * self.rc_range)
        pitch_rc = int(self.rc_center + (pitch_deg /
                       self.max_angle_deg) * self.rc_range)
        yaw_rc = int(self.rc_center + (yaw_rate_deg /
                     self.max_yaw_rate) * self.rc_range)

        # Throttle: map thrust magnitude to RC
        # Assuming hover at ~50% throttle, scale linearly
        throttle_normalized = thrust_mag / self.g_mag  # 1.0 at hover
        throttle_rc = int(1000 + throttle_normalized *
                          500)  # Hover around 1500
        throttle_rc = int(np.clip(throttle_rc, 1000, 2000))

        return {
            'roll': roll_rc,
            'pitch': pitch_rc,
            'yaw': yaw_rc,
            'throttle': throttle_rc
        }
