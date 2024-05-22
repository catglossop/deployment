from abc import ABC
import logging
from typing import Tuple, TypeVar, Union
from asyncio import Future
from typing import Optional, Type, Dict

import numpy as np

from rclpy.node import Node
from rclpy.time import Time as RosTime
from rclpy.action import ActionClient as RosActionClient
import geometry_msgs.msg as gm
from nav2_msgs.action import NavigateToPose
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer as TransformBuffer
import irobot_create_msgs.action as create_action


TwistType = Union[np.ndarray, gm.Twist, gm.TwistStamped]


def _unstamp(
    msg: Optional[TwistType], current_time: RosTime
) -> Tuple[np.ndarray, RosTime]:
    if isinstance(msg, gm.TwistStamped):
        return np.array([msg.twist.linear.x, msg.twist.angular.z]), msg.header.stamp
    elif isinstance(msg, gm.Twist):
        return np.array([msg.linear.x, msg.angular.z]), current_time
    elif isinstance(msg, np.ndarray):
        return msg, current_time
    elif msg is None:
        return np.zeros((2,)), current_time
    else:
        raise TypeError(f"Unknown type {type(msg)}")


class BaseState(ABC):
    start_time: RosTime
    last_updated_time: RosTime
    twist: np.ndarray

    def __init__(self, start_time: RosTime, initial_twist: Optional[TwistType] = None):
        self.twist, self.start_time = _unstamp(initial_twist, start_time)
        self.last_updated_time = self.start_time

    @property
    def priority(self) -> int: ...

    @property
    def timeout(self) -> Optional[float]: ...

    def update(self, *, current_time: RosTime, twist: TwistType):
        self.twist, self.last_updated_time = _unstamp(twist, current_time)

    def expired(self, current_time: RosTime):
        if self.timeout is None:
            return False
        else:
            return (current_time - self.last_updated_time).nanoseconds > self.timeout * 1e9

    def tick(self, current_time: RosTime, current_obs: Dict):
        if self.expired(current_time):
            return IdleState(current_time)
        else:
            return self

    def cancel(self):
        pass


class IdleState(BaseState):
    @property
    def priority(self):
        return 0

    @property
    def timeout(self):
        return None


class EstopState(BaseState):
    @property
    def priority(self):
        return 100

    @property
    def timeout(self):
        return None


class TeleopState(BaseState):
    @property
    def priority(self):
        return 50

    @property
    def timeout(self):
        return 0.25


class TwistTaskState(BaseState):
    @property
    def priority(self):
        return 25

    @property
    def timeout(self):
        return 0.25


class PoseTaskState(BaseState):
    def __init__(self, start_time: RosTime, initial_pose: np.ndarray, robot_frame: str, goal_frame: str):
        super().__init__(start_time)

        self.goal_pose = initial_pose
        self.tf_buffer = tf_buffer
        self.robot_frame = robot_frame
        self.goal_frame = goal_frame

    def update(self, *, current_time: RosTime, pose: np.ndarray):
        self.current_pose_goal = pose
        self.last_updated_time = current_time

    def tick(self, current_time: RosTime, current_obs: Dict):
        # Find the goal position relative to the current pose
        current_pose = current_obs["odom_pose"]
        goal_vector = goal_pose[:2] - current_pose[:2]
        goal_vector = np.array([
            [np.cos(current_pose[2]), -np.sin(current_pose[2])]
            [np.sin(current_pose[2]), np.cos(current_pose[2])]
        ]) @ goal_vector

        print(f"Pose tracking with goal vector {goal_vector}")

        # Tick the PID controller to update the twist
        self.twist = np.zeros((2,))
        return super().tick(current_time)

    @property
    def priority(self):
        return 25

    @property
    def timeout(self):
        return 0.25


class RosActionState(BaseState):
    """
    Base class for states that start and end based on the result of a ROS action.

    The ROS action should be configured to send velocities to a particular topic.
    """

    send_goal_future: Future
    done: bool

    def __init__(
        self,
        start_time: RosTime,
        clock,
        action_client: RosActionClient,
        goal,
        initial_twist: np.ndarray = np.zeros((2,)),
    ):
        super().__init__(start_time, initial_twist=initial_twist)

        self.clock = clock
        self.done = False

        # Kick off a ROS action asynchronously
        self.send_goal_future = action_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.send_goal_callback)

    def cancel(self):
        self.goal_handle.cancel_goal_async()

    def send_goal_callback(self, future: Future):
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.done = True

        goal_result_future: Future = self.goal_handle.get_result_async()
        goal_result_future.add_done_callback(self.goal_result_callback)

    def feedback_callback(self, feedback):
        if not self.done:
            self.last_updated_time = self.clock.now()

    def goal_result_callback(self, future: Future):
        self.done = True

    def expired(self, current_time: RosTime):
        return self.done or super().expired(current_time)


class IRobotActionState(RosActionState):
    """
    State for handling IRobot actions.
    """

    @property
    def priority(self):
        return 75

    @property
    def timeout(self):
        return 10


class Nav2ActionState(RosActionState):
    """
    State for handling IRobot actions.
    """

    @property
    def priority(self):
        return 25

    @property
    def timeout(self):
        return 0.25


class DoResetState(Nav2ActionState):
    def __init__(
        self,
        start_time: RosTime,
        tf_buffer: TransformBuffer,
        tf_broadcaster: TransformBroadcaster,
        action_client: RosActionClient,
        clock,
    ):
        reset_frame = tf_buffer.lookup_transform("odom", "base_link", RosTime())
        reset_frame.header.frame_id = "odom"
        reset_frame.child_frame_id = "reset"
        tf_broadcaster.sendTransform(reset_frame)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "reset"
        goal_msg.pose.pose.position.x = -1.0

        super().__init__(start_time, clock, action_client, goal_msg)

    @property
    def priority(self):
        return 25

    @property
    def timeout(self):
        return 0.25


class IRobotDockState(IRobotActionState):
    def __init__(self, start_time: RosTime, action_client: RosActionClient, clock):
        # Issue a dock command
        super().__init__(start_time, clock, action_client, create_action.DockServo.Goal())

    @property
    def priority(self):
        return 50
    
    @property
    def timeout(self):
        return 10


class IRobotUndockState(IRobotActionState):
    def __init__(self, start_time: RosTime, action_client: RosActionClient, clock):
        # Issue a dock command
        super().__init__(start_time, clock, action_client, create_action.Undock.Goal())

    @property
    def priority(self):
        return 50
    
    @property
    def timeout(self):
        return 10


class Nav2DockState(RosActionState):
    def tick(self, current_time: RosTime, current_obs: Dict):
        if self.done:
            return IRobotDockState()
        elif self.expired(current_time):
            return IdleState(current_time)
        else:
            return self


T = TypeVar("T", bound=BaseState)


class StateMachine:
    current_state: BaseState

    def __init__(self, node: Node):
        self.clock = node.get_clock()
        self.current_state = IdleState(self.clock.now())
    
    def _change_state(self, new_state: BaseState):
        print(f"STATE CHANGE {self.current_state} -> {new_state}")
        if self.current_state is not None:
            self.current_state.cancel()
        self.current_state = new_state

    def accept_state(self, new_state: BaseState):
        # Make sure the new and previous states aren't the same type
        if isinstance(self.current_state, type(new_state)):
            logging.warning(
                f"Attempted to set state to {type(new_state)} but it is already in that state."
            )
            return False

        should_accept = (
            self.current_state.expired(new_state.start_time)
            or new_state.priority > self.current_state.priority
        )

        if should_accept:
            self._change_state(new_state)

        return should_accept

    def current_state_matches(self, state_type: Type[T]) -> Optional[T]:
        if isinstance(self.current_state, state_type):
            return self.current_state
        return None

    def try_update(self, state_type: Type[T], **kwargs) -> bool:
        if isinstance(self.current_state, state_type):
            self.current_state.update(current_time=self.clock.now(), **kwargs)
            return True
        else:
            return False

    def tick(self, current_obs: Dict):
        now = self.clock.now()
        new_state = self.current_state.tick(now, current_obs)
        if self.current_state != new_state:
            self._change_state(new_state)

        if self.current_state is None or self.current_state.expired(now):
            self._change_state(IdleState(self.clock.now()))
