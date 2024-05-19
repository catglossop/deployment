from abc import ABC
import logging
from typing import Any, Optional, Tuple, Set
from agentlace.action import ActionServer, ActionConfig
import numpy as np
import tensorflow as tf

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
from tf2_ros.buffer import Buffer as TransformBuffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster

from deployment import state_machine

observation_format = {
    "image": tf.TensorSpec((), tf.string, name="image"),
    "position": tf.TensorSpec((3,), tf.float64, name="position"),
    "orientation": tf.TensorSpec((4,), tf.float64, name="orientation"),
    "imu": tf.TensorSpec((6,), tf.float64, name="imu"),
    "linear_velocity": tf.TensorSpec((3,), tf.float64, name="linear_velocity"),
    "angular_velocity": tf.TensorSpec((3,), tf.float64, name="angular_velocity"),
    "crash": tf.TensorSpec((), tf.bool, name="crash"),
    "stuck": tf.TensorSpec((), tf.bool, name="stuck"),
    "action_state": tf.TensorSpec((), tf.string, name="action_state"),
    "last_action_vw": tf.TensorSpec((2,), tf.float64, name="last_action_vw"),
    "last_action_pose": tf.TensorSpec((3,), tf.float64, name="last_action_pose"),
}

action_config = ActionConfig(
    port_number=1111,
    action_keys=["action_vw", "action_pose", "reset", "dock", "undock"],
    observation_keys=list(observation_format.keys()) + ["latest_action"],
)

class NavRobotActionServer(Node):
    def __init__(self, server_ip_address):
        super().__init__("nav_action_server")

        # TODO: Pull config from server
        self._latest_obs = {
            "image": np.array(b"", dtype=bytes),
            "position": np.zeros((3,), dtype=np.float32),
            "orientation": np.zeros((4,), dtype=np.float32),
            "imu": np.zeros((6,), dtype=np.float32),
            "linear_velocity": np.zeros((3,), dtype=np.float32),
            "angular_velocity": np.zeros((3,), dtype=np.float32),
            "crash": np.zeros((), dtype=bool),
            "stuck": np.zeros((), dtype=bool),
            "action_state": np.zeros((), dtype=np.int32),
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

        self.state_machine = state_machine.StateMachine(self)
        self.state_publisher = self.create_publisher(stdm.String, "state_machine_state", 10)
        self.twist_publisher = self.create_publisher(gm.Twist, "test_cmd_vel", 10)
        self.teleop_twist_callback = self.create_subscription(
            gm.Twist,
            "teleop_vel",
            self.receive_teleop_twist_callback,
            10,
        )
        self.nav2_twist_callback = self.create_subscription(
            gm.Twist,
            "nav2_vel",
            self.receive_nav2_twist_callback,
            10,
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

    def agentlace_obs_callback(self, keys: Set[str]):
        return {k: self._latest_obs[k] for k in keys}

    def agentlace_act_callback(self, key: str, payload: Any):
        if key == "action_vw":
            result = {"success": self.receive_vw_action_callback(payload)}
        elif key == "action_pose":
            result = {"success": self.receive_pose_action_callback(payload)}
        elif key == "reset":
            result = {"success": self.start_reset_callback()}
        else:
            result = {"success": False}
        
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
        if not self.state_machine.try_update(
            state_machine.TwistTaskState, twist=command
        ):
            self.state_machine.accept_state(
                state_machine.TwistTaskState(
                    self.get_clock().now(),
                    initial_twist=command,
                )
            )

    def receive_pose_action_callback(self, command: np.ndarray):
        if self.state_machine.try_update(state_machine.PoseTaskState, twist=command):
            return True
        else:
            return self.state_machine.accept_state(
                state_machine.PoseTaskState(
                    self.get_clock().now(),
                    initial_pose=command,
                )
            )

    def start_reset_callback(self):
        """
        Return True if a new reset was accepted or there is an existing reset in progress.
        Return False if there is an existing reset that has completed.
        """
        if self.last_agentlace_action_key == "reset":
            current_state = self.state_machine.current_state_matches(state_machine.DoResetState)
            if current_state is None:
                return False
            else:
                return current_state.done
        else:
            return self.state_machine.accept_state(
                state_machine.DoResetState(
                    self.get_clock().now(),
                    self.tf_buffer,
                    self.tf_broadcaster,
                    self.nav2_action_client,
                    self.get_clock(),
                )
            )

    def start_dock_callback(self):
        raise NotImplementedError()

    def start_undock_callback(self):
        raise NotImplementedError()

    def tick(self):
        self.state_machine.tick()
        self.republish()
        self.state_publisher.publish(stdm.String(data=f"{type(self.state_machine.current_state)}"))

    def republish(self):
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
