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
from sensor_msgs.msg import Imu, Image
from nav_msgs.msg import Odometry

# MAVROS messages / services
from mavros_msgs.msg import AttitudeTarget, State
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode as SetModeSrv
from mavros_msgs.srv import SetModeRequest

# Visualization
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Vector3, Quaternion

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


def _quat_to_euler_from_matrix(R):
    """Extract (roll, pitch, yaw) from a 3x3 rotation matrix (ZYX convention)."""
    pitch = -np.arcsin(np.clip(R[2, 0], -1.0, 1.0))
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return float(roll), float(pitch), float(yaw)


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
        config_path,
        depth_use_ros=False,
    ):
        rospy.init_node("mavros_mapper", anonymous=False)
        
        # ---- Parameters
        self.target_position = np.array([2.0, 0.0, 1.0], dtype=np.float32)
        self.margin = 1.0
        self.max_speed = 1.0

        # ---- Load optional YAML config -----------------------------------
        self._load_config(config_path)

        # ---- Sub-modules -------------------------------------------------
        if depth_use_ros:
            depth_configs = {"topic_name": "/depth_camera", "publish_preprocessed": True}
            self.depth_camera = DepthCamera(use_ros=True, configs=depth_configs)
        else:
            depth_configs = {"width": 480, "height": 270, "fps": 15}
            self.depth_camera = DepthCamera(use_ros=False, configs=depth_configs)

        self.model = NavigationModel(self.model_path)

        # ---- Internal state ----------------------------------------------
        self.g_std = np.array([0.0, 0.0, 9.80665], dtype=np.float32)
        self.position = np.zeros(3, dtype=np.float32)
        self.velocity = np.zeros(3, dtype=np.float32)   # ENU world frame

        # Attitude from IMU (quaternion kept for rotation matrix)
        self._imu_quat = None          # geometry_msgs.Quaternion
        self._odom_quat = None         # geometry_msgs.Quaternion (fallback if IMU not available)
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
        self._hover_thrust = 0.58
        self._thrust_scale = self._hover_thrust / 9.80665

        # ---- Position hold state -----------------------------------------
        self._hold_position = None   # captured when entering hold mode
        self._hold_yaw = 0.0

        # ---- Publishers ---------------------------------------------------
        self.attitude_pub = rospy.Publisher(
            "/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10
        )
        self.position_pub = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=10
        )
        self.depth_image_pub = rospy.Publisher(
            "/depth_image/preprocessed", Image, queue_size=10
        )
        self.attitude_viz_pub = rospy.Publisher(
            "/mavros_mapper/attitude_target_viz", MarkerArray, queue_size=1
        )

        # ---- Subscribers --------------------------------------------------
        # rospy.Subscriber(
        #     "/mavros/imu/data", Imu, self._imu_cb,
        #     queue_size=1, tcp_nodelay=True
        # )
        rospy.Subscriber(
            "/mavros/local_position/odom", Odometry, self._odometry_cb,
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
            rospy.Duration(secs=0, nsecs=int(1e9 / self.control_freq)), self._control_iteration
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
        if "max_speed" in cfg:
            self.max_speed = float(cfg["max_speed"])
        if "no_odom" in cfg:
            self.no_odom = bool(cfg["no_odom"])
        if "model_path" in cfg:
            self.model_path = str(cfg["model_path"])

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------
    def _imu_cb(self, msg):
        self._imu_quat = msg.orientation
        self._roll, self._pitch, self._yaw = _quat_to_euler(msg.orientation)
    
    def _odometry_cb(self, msg):
        if not self.no_odom:
            self.position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z], dtype=np.float32)
            self.velocity = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z], dtype=np.float32)
        self._odom_quat = msg.pose.pose.orientation
        self._roll, self._pitch, self._yaw = _quat_to_euler(self._odom_quat)

    def _local_pose_cb(self, msg):
        p = msg.pose.position
        self.position = np.array([p.x, p.y, p.z], dtype=np.float32)

    def _velocity_body_cb(self, msg):
        v = msg.twist.linear
        # MAVROS velocity_body is in body-FRD. We store world-frame velocity
        # by rotating back via the current attitude.
        v_body = np.array([v.x, v.y, v.z], dtype=np.float32)
        # if self._imu_quat is not None:
        #     R = _quat_to_rotation_matrix(self._imu_quat)
        #     self.velocity = R @ v_body
        if self._odom_quat is not None:
            R = _quat_to_rotation_matrix(self._odom_quat)
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
        # if self._imu_quat is not None:
        #     return _quat_to_rotation_matrix(self._imu_quat)
        
        R = _quat_to_rotation_matrix(self._odom_quat)
        env_R = R.copy()
        fwd = R[:, 0].copy()  # body x-axis as initial forward direction
        up = np.zeros_like(fwd)
        fwd[2] = 0
        up[2] = 1
        fwd = fwd / np.linalg.norm(fwd)
        R = np.stack([fwd, np.cross(up, fwd), up], -1)
        
        return R, env_R
        # Fallback: build from euler
        # roll, pitch, yaw = self._roll, self._pitch, self._yaw
        # cr, sr = np.cos(roll), np.sin(roll)
        # cp, sp = np.cos(pitch), np.sin(pitch)
        # cy, sy = np.cos(yaw), np.sin(yaw)
        # return np.array([
        #     [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        #     [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        #     [-sp,   cp*sr,            cp*cr],
        # ], dtype=np.float32)

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
        R, env_R = self._compute_rotation_matrix()
        # R_local = self._compute_local_frame_rotation()
        
        self.forward = R[:, 0].copy()  # body x-axis as forward direction

        target_v_raw = self.target_position - self.position
        target_v_norm = np.linalg.norm(target_v_raw)
        target_v = target_v_raw / target_v_norm * self.max_speed
        # if target_v_norm > 1e-6:
        #     target_v = (target_v_raw / target_v_norm) * min(
        #         target_v_norm, self.max_speed
        #     )
        # else:
        #     target_v = np.zeros(3, dtype=np.float32)

        # local_v = R_local.T @ self.velocity
        # target_v_local = R_local.T @ target_v
        # up_vector = R[:, 2]

        # observation = np.concatenate([
        #     local_v,
        #     target_v_local,
        #     up_vector,
        #     [self.margin],
        # ]).astype(np.float32)
        
        observation = np.concatenate([
            target_v @ R,
            env_R[:, 2],  # up vector in world frame
            [self.margin]
        ])
        
        if not self.no_odom:
            global_v = self.velocity @ env_R.T
            observation = np.insert(observation, 0, global_v @ R)  # prepend current velocity in world frame

        return observation[np.newaxis, :].astype(np.float32)  # (1, 7) or (1, 10)

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
    def _publish_attitude_target(self, orientation_wxyz: Quaternion, thrust):
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

        msg.orientation.w = orientation_wxyz.w
        msg.orientation.x = orientation_wxyz.x
        msg.orientation.y = orientation_wxyz.y
        msg.orientation.z = orientation_wxyz.z
        msg.body_rate.x = 0.0
        msg.body_rate.y = 0.0
        msg.body_rate.z = 0.0
        msg.thrust = float(np.clip(thrust, 0.0, 1.0))
        self.attitude_pub.publish(msg)

    def _publish_hover_attitude(self):
        """Publish a level hover attitude (used when not in AUTO)."""
        self._publish_attitude_target((1.0, 0.0, 0.0, 0.0), self._hover_thrust)

    def _publish_position_hold(self):
        """Publish a position setpoint to hold the current position.

        On first call the current position and yaw are captured as the
        hold target.  Subsequent calls keep publishing that same target
        so PX4's position controller maintains the drone in place —
        similar to POSCTL behaviour.
        """
        if self._hold_position is None:
            self._hold_position = self.position.copy()
            self._hold_yaw = self._yaw
            rospy.loginfo(
                "Position hold captured: [%.2f, %.2f, %.2f] yaw=%.2f"
                % (self._hold_position[0], self._hold_position[1],
                   self._hold_position[2], self._hold_yaw)
            )

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(self._hold_position[0])
        msg.pose.position.y = float(self._hold_position[1])
        msg.pose.position.z = float(self._hold_position[2])

        # Convert hold yaw to quaternion (roll=0, pitch=0)
        half_yaw = self._hold_yaw / 2.0
        msg.pose.orientation.w = float(np.cos(half_yaw))
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = float(np.sin(half_yaw))

        self.position_pub.publish(msg)
        
    def _publish_depth_image(self, image: np.ndarray):
        """Publish the preprocessed depth image as a ROS Image message."""
        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "depth_camera"
        msg.height = image.shape[2]  # 12
        msg.width = image.shape[3]   # 16
        msg.encoding = "32FC1"
        msg.is_bigendian = 0
        msg.step = image.shape[3] * 4  # width * sizeof(float32)
        msg.data = image.astype(np.float32).tobytes()
        self.depth_image_pub.publish(msg)

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
        
        # Capture depth frame
        image = self.depth_camera.capture_and_preprocess()  # (1,1,12,16)
        if image is None:
            rospy.logwarn_throttle(2.0, "Depth frame not available")
            self._publish_position_hold()
            return
        
        self._publish_depth_image(image)  # For debugging / visualization
        
        rospy.logwarn_throttle(2.0, "Current mode: %s, copter state: %s" % (self._state.mode, self._copter_state))

        # if self._copter_state != "AUTO":
        #     rospy.logwarn_throttle(2.0, "Current mode %s not OFFBOARD, sending hold command" % self._state.mode)
        #     self._publish_position_hold()
        #     self._forward = None
        #     self.reset_hidden_state()
        #     return
        
        # Active navigation — clear hold position so it re-captures on next hold
        self._hold_position = None
    
        # 1. Build observation
        observation = self._build_observation()  # (1, 10)

        # 2. Run model
        action = self.model.run(image, observation)  # (1, 6)

        # 3. Action -> acceleration (world frame)
        R, _ = self._compute_rotation_matrix()
        acc = self.model.action_to_acceleration(action, R, self.g_std)

        thrust_mag = float(np.linalg.norm(acc))
        up_vec = acc / thrust_mag

        # Initialise forward direction from current body x-axis
        if self._forward is None:
            self._forward = R[:, 0]
        
        target_v_raw = self.target_position - self.position
        target_v_norm = np.linalg.norm(target_v_raw)
        target_v = target_v_raw / target_v_norm * self.max_speed
        
        self._forward = self._forward * 5 + target_v
        self._forward[2] = (self._forward[0] * up_vec[0] + self._forward[1] * up_vec[1]) / -up_vec[2]
        self._forward /= np.linalg.norm(self._forward)
        left_vec = np.cross(up_vec, self._forward)
        w, x, y, z = _rotation_matrix_to_quaternion(np.stack([
            self._forward, left_vec, up_vec
        ], 1))
        
        # Normalise thrust for PX4 [0, 1]
        thrust_normalised = thrust_mag * self._thrust_scale

        # 5b. Visualize attitude & thrust target before publishing
        self._publish_attitude_viz(_quat_to_rotation_matrix(Quaternion(x, y, z, w)), thrust_normalised)

        # 6. Publish attitude target
        self._publish_attitude_target(Quaternion(w, x, y, z), thrust_normalised)
        
        # self._publish_position_hold()  # For hover / position-hold testing

    # ------------------------------------------------------------------
    # Attitude target visualization
    # ------------------------------------------------------------------
    def _publish_attitude_viz(self, R_des, thrust_normalised):
        """Publish MarkerArray showing desired orientation axes and thrust.

        Publishes to ``/mavros_mapper/attitude_target_viz`` for RViz:
          - Red arrow   -> body X (forward)
          - Green arrow  -> body Y (left)
          - Blue arrow   -> body Z (up)
          - Yellow arrow -> thrust vector (length proportional to thrust)
        """
        markers = MarkerArray()
        stamp = rospy.Time.now()
        pos = self.position  # drone position in ENU

        axis_length = 0.6  # metres
        axis_diam = 0.04

        axis_colors = [
            ColorRGBA(1.0, 0.2, 0.2, 0.9),  # X – red
            ColorRGBA(0.2, 1.0, 0.2, 0.9),  # Y – green
            ColorRGBA(0.2, 0.2, 1.0, 0.9),  # Z – blue
        ]
        axis_labels = ["des_X", "des_Y", "des_Z"]

        for i in range(3):
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = "map"
            m.ns = "attitude_target"
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD

            start = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
            end_pt = pos + R_des[:, i] * axis_length
            end = Point(x=float(end_pt[0]), y=float(end_pt[1]), z=float(end_pt[2]))
            m.points = [start, end]

            m.scale = Vector3(x=axis_diam, y=axis_diam * 1.8, z=0.0)
            m.color = axis_colors[i]
            m.lifetime = rospy.Duration(0.2)
            markers.markers.append(m)

        # Thrust vector (along desired up, length = thrust_normalised)
        thrust_dir = R_des[:, 2]  # desired body Z
        thrust_len = float(thrust_normalised) * 1.5  # scale for visibility
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = "map"
        m.ns = "attitude_target"
        m.id = 3
        m.type = Marker.ARROW
        m.action = Marker.ADD

        start = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
        end_pt = pos + thrust_dir * thrust_len
        end = Point(x=float(end_pt[0]), y=float(end_pt[1]), z=float(end_pt[2]))
        m.points = [start, end]

        m.scale = Vector3(x=0.06, y=0.10, z=0.0)
        m.color = ColorRGBA(1.0, 0.9, 0.0, 0.9)  # yellow
        m.lifetime = rospy.Duration(0.2)
        markers.markers.append(m)

        # Text overlay: thrust value
        txt = Marker()
        txt.header.stamp = stamp
        txt.header.frame_id = "map"
        txt.ns = "attitude_target"
        txt.id = 4
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.pose.position.x = float(pos[0])
        txt.pose.position.y = float(pos[1])
        txt.pose.position.z = float(pos[2]) + 0.8
        txt.scale.z = 0.15
        txt.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)

        roll_d, pitch_d, _ = _quat_to_euler_from_matrix(R_des)
        txt.text = "T=%.2f R=%.1f P=%.1f" % (
            thrust_normalised,
            np.degrees(roll_d),
            np.degrees(pitch_d),
        )
        txt.lifetime = rospy.Duration(0.2)
        markers.markers.append(txt)

        self.attitude_viz_pub.publish(markers)

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
    # argparser.add_argument(
    #     "--model_path", type=str, default="base.onnx",
    #     help="Path to the ONNX model file",
    # )
    # argparser.add_argument(
    #     "--control_freq", type=float, default=100.0,
    #     help="Control loop frequency in Hz",
    # )
    argparser.add_argument(
        "--depth_ros", action="store_true",
        help="Use ROS topic for depth instead of native RealSense SDK",
    )
    cli_args = argparser.parse_args()

    mapper = MavrosMapper(
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
