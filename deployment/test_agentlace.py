from agentlace.action import ActionClient, ActionConfig
import numpy as np
import tensorflow as tf

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

def main():
    robot = ActionClient(
        "127.0.0.1",
        action_config,
    )

    robot.act("reset", np.array([0.0, 1.0]))

if __name__ == "__main__":
    main()