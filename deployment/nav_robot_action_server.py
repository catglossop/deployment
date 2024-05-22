from abc import ABC
import logging
from typing import Any, Optional, Tuple, Set, Callable
from agentlace.action import ActionServer, ActionConfig
import numpy as np
import tensorflow as tf

from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.action import ActionClient as RosActionClient
import rclpy
from rclpy.time import Time as RosTime
from rclpy.node import Node
import geometry_msgs.msg as gm
import nav_msgs.msg as nm
import sensor_msgs.msg as sm
import std_msgs.msg as stdm
from nav2_msgs.action import NavigateToPose
from irobot_create_msgs.action import Undock, DockServo
from tf2_ros import Buffer as TransformBuffer, TransformListener, TransformBroadcaster, LookupException, ConnectivityException, ExtrapolationException
import tf_transformations
from tf2_geometry_msgs import do_transform_pose_stamped

from deployment import state_machine

observation_format = {
    # Raw sensor
    "image": tf.TensorSpec((), tf.string, name="image"),
    "imu_accel": tf.TensorSpec((3,), tf.float32, name="imu_accel"),
    "imu_gyro": tf.TensorSpec((3,), tf.float32, name="imu_gyro"),
    "crash": tf.TensorSpec((), tf.bool, name="crash"),
    "odom_pose": tf.TensorSpec((3,), tf.float32, name="odom_pose"),
    "linear_velocity": tf.TensorSpec((3,), tf.float32, name="linear_velocity"),
    "angular_velocity": tf.TensorSpec((3,), tf.float32, name="angular_velocity"),

    # Estimator
    "position": tf.TensorSpec((3,), tf.float32, name="position"),
    "orientation": tf.TensorSpec((4,), tf.float32, name="orientation"),
    "pose_std": tf.TensorSpec((6,), tf.float32, name="pose_std"),

    # State machine and action
    "action_state": tf.TensorSpec((), tf.string, name="action_state"),
    "last_action_linear": tf.TensorSpec((3,), tf.float32, name="last_action_linear"),
    "last_action_angular": tf.TensorSpec((3,), tf.float32, name="last_action_angular"),
}

action_config = ActionConfig(
    port_number=1111,
    action_keys=["action_vw", "action_pose", "reset", "dock", "undock"],
    observation_keys=list(observation_format.keys()) + ["latest_action"],
)

def transform_odometry_to_map(odometry_msg: nm.Odometry, tf_buffer):
    try:
        # Create a PoseStamped object from the Odometry message
        pose_stamped = gm.PoseStamped()
        pose_stamped.header = odometry_msg.header
        pose_stamped.pose = odometry_msg.pose.pose

        # Lookup the latest transform from the "odom" frame to the "map" frame
        transform = tf_buffer.lookup_transform('map', odometry_msg.header.frame_id, RosTime())

        # Transform the PoseStamped object to the "map" frame
        transformed_pose = do_transform_pose_stamped(pose_stamped, transform)

        # Create a new Odometry message for the transformed pose
        transformed_odometry = nm.Odometry()
        transformed_odometry.header = transformed_pose.header
        transformed_odometry.child_frame_id = 'base_link'  # or whichever frame your robot's odometry refers to
        transformed_odometry.pose.pose = transformed_pose.pose
        transformed_odometry.twist = odometry_msg.twist  # assuming twist remains in the original frame

        return transformed_odometry

    except (LookupException, ConnectivityException, ExtrapolationException) as e:
        print(f"Transform error: {e}")
        return None

class NavRobotActionServer(Node):
    def __init__(self, server_ip_address):
        super().__init__("nav_action_server")

        # TODO: Pull config from server
        self._latest_obs = {
            "image": np.array(b"", dtype=bytes),
            "imu_accel": np.zeros((3,), dtype=np.float32),
            "imu_gyro": np.zeros((3,), dtype=np.float32),
            "crash": np.zeros((), dtype=bool),
            "odom_pose": np.zeros((3,), dtype=np.float32),
            "linear_velocity": np.zeros((3,), dtype=np.float32),
            "angular_velocity": np.zeros((3,), dtype=np.float32),

            "position": np.zeros((3,), dtype=np.float32),
            "orientation": np.zeros((4,), dtype=np.float32),
            "pose_std": np.zeros((6,), dtype=np.float32),

            "action_state_source": np.zeros((), dtype=str),
            "last_linear_velocity": np.zeros((3,), dtype=str),
            "last_angular_velocity": np.zeros((3,), dtype=str),
        }

        # ROS parameters
        self.tick_rate = self.declare_parameter("tick_rate", 10)

        self.nav2_action_client = RosActionClient(
            self, NavigateToPose, "navigate_to_pose"
        )
        self.dock_action_client = RosActionClient(self, DockServo, "dock")
        self.undock_action_client = RosActionClient(self, Undock, "undock")

        self.tf_buffer = TransformBuffer()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.reset_frame = gm.TransformStamped()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1,
        )

        self.state_machine = state_machine.StateMachine(self)
        self.state_publisher = self.create_publisher(stdm.String, "state_machine_state", 10)
        self.twist_publisher = self.create_publisher(gm.Twist, "cmd_vel", 10)
        self.teleop_twist_sub = self.create_subscription(
            gm.Twist,
            "teleop_vel",
            self.receive_teleop_twist_callback,
            10,
        )
        self.nav2_twist_sub = self.create_subscription(
            gm.Twist,
            "nav2_vel",
            self.receive_nav2_twist_callback,
            10,
        )

        # Sensor subscriptions
        self.image_sub = self.create_subscription(
            sm.CompressedImage,
            "/front/image_raw/compressed",
            self.image_callback,
            10,
        )
        self.amcl_pose_sub = self.create_subscription(
            gm.PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_pose_callback,
            10,
        )
        self.odom_sub = self.create_subscription(
            nm.Odometry,
            "/odom",
            self.odom_callback,
            best_effort_qos,
        )
        self.imu_sub = self.create_subscription(
            sm.Imu,
            "/imu",
            self.imu_callback,
            best_effort_qos,
        )

        self.action_server = ActionServer(
            config=action_config,
            obs_callback=self.agentlace_obs_callback,
            act_callback=self.agentlace_act_callback,
        )

        self.state_machine_timer = self.create_timer(
            1 / self.tick_rate.value, self.tick
        )

        self.last_agentlace_action_key = None

        # Start running
        self.action_server.start(threaded=True)
    
    def image_callback(self, image: sm.CompressedImage):
        self._latest_obs["image"] = np.array(image.data, dtype=bytes)
    
    def amcl_pose_callback(self, pose: gm.PoseWithCovarianceStamped):
        self._latest_obs["pose_std"] = np.sqrt(np.diag(np.array(pose.pose.covariance).reshape((6, 6)))).astype(np.float32)
    
    def odom_callback(self, odom: nm.Odometry):
        self._latest_obs["odom_pose"] = np.array([
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            tf_transformations.euler_from_quaternion([
                odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z,
                odom.pose.pose.orientation.w,
            ])[-1],
        ], dtype=np.float32)

        self._latest_obs["linear_velocity"] = np.array([
            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y,
            odom.twist.twist.linear.z,
        ], dtype=np.float32)

        self._latest_obs["angular_velocity"] = np.array([
            odom.twist.twist.angular.x,
            odom.twist.twist.angular.y,
            odom.twist.twist.angular.z,
        ], dtype=np.float32)

        odom_map = transform_odometry_to_map(odom, self.tf_buffer)

        if odom_map is not None:
            self._latest_obs["position"] = np.array(
                [
                    odom_map.pose.pose.position.x,
                    odom_map.pose.pose.position.y,
                    odom_map.pose.pose.position.z,
                ],
                dtype=np.float32
            )
            self._latest_obs["orientation"] = np.array(
                [
                    odom_map.pose.pose.orientation.x,
                    odom_map.pose.pose.orientation.y,
                    odom_map.pose.pose.orientation.z,
                    odom_map.pose.pose.orientation.w,
                ],
                dtype=np.float32
            )

    
    def imu_callback(self, imu: sm.Imu):
        self._latest_obs["imu_accel"] = np.array([
            imu.linear_acceleration.x,
            imu.linear_acceleration.y,
            imu.linear_acceleration.z,
        ], dtype=np.float32)

        self._latest_obs["imu_gyro"] = np.array([
            imu.angular_velocity.x,
            imu.angular_velocity.y,
            imu.angular_velocity.z,
        ], dtype=np.float32)

    def agentlace_obs_callback(self, keys: Set[str]):
        return {k: self._latest_obs[k] for k in keys}

    def agentlace_act_callback(self, key: str, payload: Any):
        if key == "action_vw":
            result = self.receive_vw_action_callback(payload)
        elif key == "action_pose":
            result = self.receive_pose_action_callback(payload)
        elif key == "reset":
            result = self.start_reset_callback()
        elif key == "dock":
            result = self.start_dock_callback()
        elif key == "undock":
            result = self.start_undock_callback()
        else:
            result = {"running": False, "reason": f"Unknown key {key}"}
        
        self.last_agentlace_action_key = key

        return result

    def receive_teleop_twist_callback(self, command: state_machine.TwistType):
        if not self.state_machine.try_update(state_machine.TeleopState, twist=command):
            return self.state_machine.accept_state(
                state_machine.TeleopState(
                    self.get_clock().now(),
                    initial_twist=command,
                )
            )

    def receive_nav2_twist_callback(self, command: state_machine.TwistType):
        if not self.state_machine.try_update(
            state_machine.Nav2ActionState, twist=command
        ):
            logging.info("Nav2 command received while not in Nav2ActionState")

    def receive_vw_action_callback(self, command: np.ndarray):
        if self.state_machine.try_update(state_machine.TwistTaskState, twist=command):
            return {"running": True, "reason": "already running"}
        else:
            accepted = self.state_machine.accept_state(
                state_machine.TwistTaskState(
                    self.get_clock().now(),
                    initial_pose=command,
                )
            )
            if accepted:
                return {"running": True, "reason": "started action"}
            else:
                return {"running": False, "reason": f"current state is {self.state_machine.current_state}"}

    def _do_start_action(self, make_state_machine_state: Callable):
        """
        Return True if a new reset was accepted or there is an existing reset in progress.
        Return False if there is an existing reset that has completed.
        """
        if self.last_agentlace_action_key == "reset":
            current_state = self.state_machine.current_state_matches(state_machine.DoResetState)
            if current_state is None:
                return {"running": False, "reason": "state is idle"}
            elif current_state.done:
                return {"running": False, "reason": "completed reset action"}
            else:
                return {"running": True, "reason": "already running"}
        else:
            accepted = self.state_machine.accept_state(make_state_machine_state())
            if accepted:
                return {"running": True, "reason": "started action"}
            else:
                return {"running": False, "reason": f"current state is {self.state_machine.current_state}"}

    def receive_pose_action_callback(self, command: np.ndarray):
        if self.state_machine.try_update(state_machine.PoseTaskState, pose=command):
            return {"running": True, "reason": "already running"}
        else:
            accepted = self.state_machine.accept_state(
                state_machine.PoseTaskState(
                    self.get_clock().now(),
                    initial_pose=command,
                )
            )
            if accepted:
                return {"running": True, "reason": "started action"}
            else:
                return {"running": False, "reason": f"current state is {self.state_machine.current_state}"}

    def start_reset_callback(self):
        return self._do_start_action(
            lambda: state_machine.DoResetState(
                self.get_clock().now(),
                self.tf_buffer,
                self.tf_broadcaster,
                self.nav2_action_client,
                self.get_clock(),
            )
        )

    def start_dock_callback(self):
        return self._do_start_action(
            lambda: state_machine.IRobotDockState(
                self.get_clock().now(),
                self.dock_action_client,
                self.get_clock(),
            )
        )

    def start_undock_callback(self):
        return self._do_start_action(
            lambda: state_machine.IRobotUndockState(
                self.get_clock().now(),
                self.dock_action_client,
                self.get_clock(),
            )
        )

    def tick(self):
        self.state_machine.tick(self._latest_obs)
        self.republish()
        self.state_publisher.publish(stdm.String(data=type(self.state_machine.current_state).__name__))

    def republish(self):
        self._latest_obs["action_state_source"] = type(self.state_machine.current_state).__name__
        self._latest_obs["last_action_linear"] = np.array([self.state_machine.current_state.twist[0], 0.0, 0.0], dtype=np.float32)
        self._latest_obs["last_action_angular"] = np.array([0.0, 0.0, self.state_machine.current_state.twist[1]], dtype=np.float32)

        twist_msg = gm.Twist()
        twist_msg.linear.x = float(self.state_machine.current_state.twist[0])
        twist_msg.angular.z = float(self.state_machine.current_state.twist[1])
        self.twist_publisher.publish(twist_msg)


if __name__ == "__main__":
    rclpy.init()

    import logging
    logging.basicConfig(level=logging.WARNING)

    node = NavRobotActionServer("127.0.0.1")

    rclpy.spin(node)
