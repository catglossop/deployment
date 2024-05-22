import time
from agentlace.action import ActionClient, ActionConfig
import numpy as np
import tensorflow as tf

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
    "last_action_vw": tf.TensorSpec((2,), tf.float32, name="last_action_vw"),
    "last_action_pose": tf.TensorSpec((3,), tf.float32, name="last_action_pose"),
}

action_config = ActionConfig(
    port_number=1111,
    action_keys=["action_vw", "action_pose", "reset", "dock", "undock"],
    observation_keys=list(observation_format.keys()) + ["latest_action"],
)

def main():
    robot = ActionClient(
        "127.0.0.1",
        action_config,
    )

    while True:
        print("Docking: ", robot.act("dock", np.zeros(1)))
        # obs = robot.obs({"position", "orientation", "pose_std", "odom_pose", "linear_velocity", "angular_velocity"})
        # print("-"*80)
        # print(f"odom pose: {obs['odom_pose']}")
        # print(f"amcl position: {obs['position']}\tamcl std: {obs['pose_std'][:2]}")
        # print(f"linear velocity: {obs['linear_velocity']}\tangular velocity: {obs['angular_velocity']}")
        time.sleep(0.5)

if __name__ == "__main__":
    main()