"""
MavrosMapper: Navigation controller using MAVROS (ROS 1 Noetic) as the
flight-controller interface. Mirrors the MSPMapper architecture but replaces
the MSP serial protocol with MAVROS topics and services.

Subscriptions (input):
    /mavros/imu/data                 – attitude (quaternion)
    /mavros/local_position/pose      – local position (ENU)
    /mavros/local_position/velocity_body – body-frame velocity
    /mavros/state                    – armed / mode / connected

Publications (output):
    /mavros/setpoint_raw/attitude    – attitude target (orientation + collective thrust)

Services (optional helpers):
    /mavros/cmd/arming               – arm / disarm
    /mavros/set_mode                 – set flight mode
"""

from __future__ import annotations

import argparse
import signal
import sys
from pathlib import Path

import numpy as np
import rospy

# Standard ROS messages
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu

# MAVROS messages / services
from mavros_msgs.msg import AttitudeTarget, State
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode as SetModeSrv
from mavros_msgs.srv import SetModeRequest

# Project-local modules
from Model import NavigationModel
from DepthCamera import DepthCamera


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _quat_to_euler(q):
    """Convert geometry_msgs.Quaternion -> (roll, pitch, yaw) in radians.

    Uses the ZYX (aerospace) convention.
    """
    x, y, z, w = q.x, q.y, q.z, q.w

    # Roll (x-axis)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis)
    sinp = 2.0 * (w * y - z * x)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)

    # Yaw (z-axis)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return float(roll), float(pitch), float(yaw)


def _quat_to_rotation_matrix(q):
    """Convert geometry_msgs.Quaternion -> 3x3 rotation matrix."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),       1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),       2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ], dtype=np.float32)


def _rotation_matrix_to_quaternion(R):
    """Convert 3x3 rotation matrix -> quaternion (w, x, y, z).

    Uses Shepperd's method for numerical stability.
    """
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (float(w), float(x), float(y), float(z))


# ---------------------------------------------------------------------------
# MavrosMapper
# ---------------------------------------------------------------------------

class MavrosMapper(object):
    """ROS node that reads MAVROS telemetry, runs the navigation model, and
    publishes attitude target commands through MAVROS."""

    def __init__(
        self,
        model_path=None,
        config_path=None,
        depth_use_ros=False,
    ):
        rospy.init_node("mavros_mapper", anonymous=False)

        # ---- Load optional YAML config -----------------------------------
        self._load_config(config_path)

        # ---- Sub-modules -------------------------------------------------
        if depth_use_ros:
            depth_configs = {"topic_name": "/depth_camera"}
            self.depth_camera = DepthCamera(use_ros=True, configs=depth_configs)
        else:
            depth_configs = {"width": 480, "height": 270, "fps": 15}
            self.depth_camera = DepthCamera(use_ros=False, configs=depth_configs)

        self.model = NavigationModel(model_path)

        # ---- Internal state ----------------------------------------------
        self.g_std = np.array([0.0, 0.0, -9.80665], dtype=np.float32)

        self.velocity = np.zeros(3, dtype=np.float32)   # ENU world frame
        self.position = np.zeros(3, dtype=np.float32)
        self.target_position = np.array([5.0, 0.0, 1.5], dtype=np.float32)
        self.margin = 1.0
        self.max_speed = 3.0

        # Attitude from IMU (quaternion kept for rotation matrix)
        self._imu_quat = None          # geometry_msgs.Quaternion
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0

        # MAVROS state
        self._state = State()

        # Copter state machine
        self._copter_state = None

        # Forward direction vector for yaw tracking (initialised on first use)
        self._forward = None

        # Thrust normalisation: hover_thrust / g  maps 1 g -> hover_thrust
        self._hover_thrust = 0.5
        self._thrust_scale = self._hover_thrust / 9.80665

        # ---- Publishers ---------------------------------------------------
        self.attitude_pub = rospy.Publisher(
            "/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10
        )

        # ---- Subscribers --------------------------------------------------
        rospy.Subscriber(
            "/mavros/imu/data", Imu, self._imu_cb,
            queue_size=1, tcp_nodelay=True
        )
        rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self._local_pose_cb,
            queue_size=1, tcp_nodelay=True
        )
        rospy.Subscriber(
            "/mavros/local_position/velocity_body", TwistStamped,
            self._velocity_body_cb, queue_size=1, tcp_nodelay=True
        )
        rospy.Subscriber(
            "/mavros/state", State, self._state_cb, queue_size=1
        )

        # ---- Service proxies (arm / set_mode) ----------------------------
        self._arm_proxy = None
        self._set_mode_proxy = None

        # ---- Control timer ------------------------------------------------
        self._control_timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_freq), self._control_iteration
        )

        rospy.loginfo(
            "MavrosMapper initialised - control @ %.1f Hz" % self.control_freq
        )

    # ------------------------------------------------------------------
    # Config loading
    # ------------------------------------------------------------------
    def _load_config(self, config_path):
        """Load YAML config or set defaults."""
        self.control_freq = 100.0
        if config_path is None:
            return
        path = Path(config_path)
        if not path.is_file():
            rospy.logwarn("Config file not found: %s" % config_path)
            return
        import yaml
        with open(path, "r") as f:
            cfg = yaml.safe_load(f) or {}
        if "control_freq" in cfg:
            self.control_freq = float(cfg["control_freq"])

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------
    def _imu_cb(self, msg):
        self._imu_quat = msg.orientation
        self._roll, self._pitch, self._yaw = _quat_to_euler(msg.orientation)

    def _local_pose_cb(self, msg):
        p = msg.pose.position
        self.position = np.array([p.x, p.y, p.z], dtype=np.float32)

    def _velocity_body_cb(self, msg):
        v = msg.twist.linear
        # MAVROS velocity_body is in body-FRD. We store world-frame velocity
        # by rotating back via the current attitude.
        v_body = np.array([v.x, v.y, v.z], dtype=np.float32)
        if self._imu_quat is not None:
            R = _quat_to_rotation_matrix(self._imu_quat)
            self.velocity = R @ v_body
        else:
            self.velocity = v_body

    def _state_cb(self, msg):
        self._state = msg

    # ------------------------------------------------------------------
    # Public setters (match MSPMapper API)
    # ------------------------------------------------------------------
    def set_target_position(self, target):
        self.target_position = np.asarray(target, dtype=np.float32)

    def update_margin(self, margin):
        self.margin = float(margin)

    def reset_hidden_state(self):
        self.model.reset_hidden_state()

    # ------------------------------------------------------------------
    # Attitude / rotation helpers
    # ------------------------------------------------------------------
    def _compute_rotation_matrix(self):
        if self._imu_quat is not None:
            return _quat_to_rotation_matrix(self._imu_quat)
        # Fallback: build from euler
        roll, pitch, yaw = self._roll, self._pitch, self._yaw
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        return np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr],
        ], dtype=np.float32)

    def _compute_local_frame_rotation(self):
        yaw = self._yaw
        cy, sy = np.cos(yaw), np.sin(yaw)
        return np.array([
            [cy, -sy, 0],
            [sy,  cy, 0],
            [0,   0,  1],
        ], dtype=np.float32)

    # ------------------------------------------------------------------
    # Observation builder
    # ------------------------------------------------------------------
    def _build_observation(self):
        R = self._compute_rotation_matrix()
        R_local = self._compute_local_frame_rotation()

        target_v_raw = self.target_position - self.position
        target_v_norm = np.linalg.norm(target_v_raw)
        if target_v_norm > 1e-6:
            target_v = (target_v_raw / target_v_norm) * min(
                target_v_norm, self.max_speed
            )
        else:
            target_v = np.zeros(3, dtype=np.float32)

        local_v = R_local.T @ self.velocity
        target_v_local = R_local.T @ target_v
        up_vector = R[:, 2]

        observation = np.concatenate([
            local_v,
            target_v_local,
            up_vector,
            [self.margin],
        ]).astype(np.float32)

        return observation[np.newaxis, :]  # (1, 10)

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------
    def _update_copter_state(self):
        """Determine copter state from the MAVROS State topic."""
        if not self._state.connected:
            self._copter_state = None
            return
        mode = self._state.mode.upper()
        if mode == "OFFBOARD":
            self._copter_state = "AUTO"
        else:
            self._copter_state = "REMOTE"

    # ------------------------------------------------------------------
    # Attitude target publishing
    # ------------------------------------------------------------------
    def _publish_attitude_target(self, orientation_wxyz, thrust):
        """Publish a mavros_msgs/AttitudeTarget message.

        Args:
            orientation_wxyz: Desired orientation as (w, x, y, z) quaternion.
            thrust: Normalised collective thrust [0, 1].
        """
        msg = AttitudeTarget()
        msg.header.stamp = rospy.Time.now()
        msg.type_mask = (
            AttitudeTarget.IGNORE_ROLL_RATE
            | AttitudeTarget.IGNORE_PITCH_RATE
            | AttitudeTarget.IGNORE_YAW_RATE
        )
        w, x, y, z = orientation_wxyz
        msg.orientation.w = float(w)
        msg.orientation.x = float(x)
        msg.orientation.y = float(y)
        msg.orientation.z = float(z)
        msg.body_rate.x = 0.0
        msg.body_rate.y = 0.0
        msg.body_rate.z = 0.0
        msg.thrust = float(np.clip(thrust, 0.0, 1.0))
        self.attitude_pub.publish(msg)

    def _publish_hover_attitude(self):
        """Publish a level hover attitude (used when not in AUTO)."""
        self._publish_attitude_target((1.0, 0.0, 0.0, 0.0), self._hover_thrust)

    # ------------------------------------------------------------------
    # Service helpers
    # ------------------------------------------------------------------
    def arm(self, value=True):
        """Request arm (True) or disarm (False) via MAVROS service."""
        try:
            if self._arm_proxy is None:
                rospy.wait_for_service("/mavros/cmd/arming", timeout=2.0)
                self._arm_proxy = rospy.ServiceProxy(
                    "/mavros/cmd/arming", CommandBool
                )
            resp = self._arm_proxy(value)
            rospy.loginfo("Arm(%s): success=%s" % (value, resp.success))
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logwarn("Arming service call failed: %s" % e)

    def set_mode(self, mode):
        """Set the FCU mode (e.g. 'OFFBOARD', 'STABILIZE')."""
        try:
            if self._set_mode_proxy is None:
                rospy.wait_for_service("/mavros/set_mode", timeout=2.0)
                self._set_mode_proxy = rospy.ServiceProxy(
                    "/mavros/set_mode", SetModeSrv
                )
            resp = self._set_mode_proxy(custom_mode=mode)
            rospy.loginfo("SetMode(%s): mode_sent=%s" % (mode, resp.mode_sent))
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logwarn("SetMode service call failed: %s" % e)

    # ------------------------------------------------------------------
    # Main control loop (timer callback)
    # ------------------------------------------------------------------
    def _control_iteration(self, event):
        self._update_copter_state()

        if self._copter_state is None:
            rospy.logwarn_throttle(5.0, "MAVROS not connected yet")
            return

        if self._copter_state != "AUTO":
            self._publish_hover_attitude()
            self._forward = None
            self.reset_hidden_state()
            return

        # Capture depth frame
        image = self.depth_camera.capture_and_preprocess()  # (1,1,12,16)
        if image is None:
            rospy.logwarn_throttle(2.0, "Depth frame not available")
            self._publish_hover_attitude()
            return

        # 1. Build observation
        observation = self._build_observation()  # (1, 10)

        # 2. Run model
        action = self.model.run(image, observation)  # (1, 6)

        # 3. Action -> acceleration (world frame)
        R_local = self._compute_local_frame_rotation()
        acc = self.model.action_to_acceleration(action, R_local, self.g_std)

        # 4. Gravity compensation -> thrust vector
        acc[2] += 9.80665

        thrust_mag = float(np.linalg.norm(acc))
        if thrust_mag < 1e-6:
            thrust_mag = 9.80665
            up_vec = np.array([0.0, 0.0, 1.0], dtype=np.float32)
        else:
            up_vec = acc / thrust_mag

        # 5. Desired orientation: build [forward, left, up] rotation matrix
        #    (same approach as PlannerBase.callback_depth)
        target_dir = self.target_position - self.position
        target_dir[2] = 0.0  # project to horizontal
        target_dir_norm = float(np.linalg.norm(target_dir))
        if target_dir_norm > 0.1:
            target_v = (target_dir / target_dir_norm) * self.max_speed
        else:
            target_v = np.array([self.max_speed, 0.0, 0.0], dtype=np.float32)

        # Initialise forward direction from current body x-axis
        if self._forward is None:
            R_body = self._compute_rotation_matrix()
            self._forward = R_body[:, 0].copy()

        # Smooth forward direction towards target velocity
        self._forward = self._forward * 5.0 + target_v
        # Project forward onto the plane perpendicular to up_vec
        if abs(up_vec[2]) > 1e-6:
            self._forward[2] = (
                -(self._forward[0] * up_vec[0] + self._forward[1] * up_vec[1])
                / up_vec[2]
            )
        fwd_norm = float(np.linalg.norm(self._forward))
        if fwd_norm > 1e-6:
            self._forward = self._forward / fwd_norm

        left_vec = np.cross(up_vec, self._forward)
        left_norm = float(np.linalg.norm(left_vec))
        if left_norm > 1e-6:
            left_vec = left_vec / left_norm

        # Desired rotation matrix -> quaternion
        R_des = np.column_stack([self._forward, left_vec, up_vec]).astype(np.float32)
        quat_wxyz = _rotation_matrix_to_quaternion(R_des)

        # Normalise thrust for PX4 [0, 1]
        thrust_normalised = thrust_mag * self._thrust_scale

        # 6. Publish attitude target
        self._publish_attitude_target(quat_wxyz, thrust_normalised)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def start_realsense(self):
        """Start the depth camera pipeline."""
        self.depth_camera.start()

    def stop_realsense(self):
        """Stop the depth camera pipeline."""
        self.depth_camera.stop()

    def spin(self):
        """Block until ROS shuts down."""
        rospy.spin()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    argparser = argparse.ArgumentParser(
        description="MavrosMapper - MAVROS-based navigation controller"
    )
    argparser.add_argument(
        "--config", type=str, default=None,
        help="Path to YAML configuration file",
    )
    argparser.add_argument(
        "--model_path", type=str, default="base.onnx",
        help="Path to the ONNX model file",
    )
    argparser.add_argument(
        "--control_freq", type=float, default=100.0,
        help="Control loop frequency in Hz",
    )
    argparser.add_argument(
        "--depth_ros", action="store_true",
        help="Use ROS topic for depth instead of native RealSense SDK",
    )
    cli_args = argparser.parse_args()

    mapper = MavrosMapper(
        model_path=cli_args.model_path,
        config_path=cli_args.config,
        depth_use_ros=cli_args.depth_ros,
    )

    # Start RealSense before spinning
    mapper.start_realsense()

    def _shutdown():
        rospy.loginfo("Shutting down...")
        mapper.stop_realsense()

    rospy.on_shutdown(_shutdown)

    try:
        mapper.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
