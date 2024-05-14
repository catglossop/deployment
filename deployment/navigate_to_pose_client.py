from action_msgs.msg import GoalStatus
from example_interfaces.action import Fibonacci

import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.node import Node   

class NavigateToPoseClient(Node)

    def __init__(self):
        super().__init__('nav_to_pose_action_client')
        self._action_client = ActionClient(self, NavigateToPose 'navigate_to_pose')

    def goal_response_callback(self, future):
        print("IN GOAL RESPONSE")
        goal_handle = future.result()
        if not goal_handle.accepted:
            print('Goal rejected :(')
            return

        print('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        print('Received feedback: {0}'.format(feedback.feedback.sequence))

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result.sequence))
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))

        self.status = GoalStatus.STATUS_SUCCEEDED

    def send_navigate_to_pose(self, pose):
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose = pose

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

def main(args=None):
    rclpy.init(args=args)

    action_client = NavigateToPoseClient()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()