import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, PoseStamped
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from irobot_create_msgs.msg import Dock
from irobot_create_msgs.action import Undock, DockServo, AudioNoteSequence
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from action_msgs.msg import GoalStatus

FREQUENCIES = [262, 262, 392, 392, 440, 440, 392]

class Orchestrator(Node): 

    def __init__(self, 
                task_nodes, 
                fallback_nodes):
        super().__init__("orchestrator")

        self.STATE = "DO_TASK"
        self.PREV_STATE = "IDLE"
        self.task_nodes = task_nodes
        self.fallback_nodes = fallback_nodes
        self.timer_period = 0.01
        self.init_pose = PoseStamped()
        self.goal_pose = self.init_pose
        self.pose_goal_status = GoalStatus.STATUS_UNKNOWN
        self.panic_song = AudioNoteVector()
        for freq in FREQUENCIES: 
            note = AudioNote()
            note.frequency = freq
            note.max_runtime.sec = 1

            self.panic_song.notes.append(note)

        self.irobot_qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT, 
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.VOLATILE,
                depth=1
        )

        self.nav_vel_msg = Twist()
        self.nav_vel_sub = self.create_subscription(
            Twist, 
            "/nav2/cmd_vel",
            self.nav_vel_callback,
        1)
        self.vel_msg = Twist()
        self.vel_pub = self.create_publisher(
            Twist,
            "/cmd_vel", 
            1
        )
        self.audio_pub = self.create_publisher(
            AudioNoteVector, 
            "/cmd_audio",
            1
        )
        # Subscribe to dock msgs
        self.dock_msg = Dock()
        self.dock_sub = self.create_subscription(
            Dock, 
            "/dock",
            self.dock_callback,
            self.irobot_qos_profile)
        self.timer = self.create_timer(self.timer_period, self.status_callback)
        self.undock_action_client = ActionClient(self, Undock, 'undock')
        self.dock_action_client = ActionClient(self, DockServo, 'dock')
        self.navigate_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
    
    def nav_vel_callback(self, vel_msg):

        self.nav_vel_msg = vel_msg

    def dock_callback(self, dock_msg):

        self.dock_msg = dock_msg

    def send_undock(self):
        goal_msg = Undock.Goal()
        
        self.undock_action_client.wait_for_server()

        return self.undock_action_client.send_goal_async(goal_msg)

    def send_dock(self):
        goal_msg = DockServo.Goal()

        self.dock_action_client.wait_for_server()

        return self.dock_action_client.send_goal_async(goal_msg)

    def goal_response_callback(self, future):
        print("IN GOAL RESPONSE")
        goal_handle = future.result()
        if not goal_handle.accepted:
            print('Goal rejected :(')
            return

        print('Goal accepted :)')

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # For now, no feedback

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result))
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))

        self.pose_goal_status = status
        if self.STATE == "NAV_TO_DOCK":
            self.STATE = "DOCK"

    def send_navigate_to_pose(self):
        goal_msg = NavigateToPose.Goal()
        self.pose_goal_status = GoalStatus.STATUS_EXECUTING

        goal_msg.pose = self.goal_pose

        self.navigate_to_pose_client.wait_for_server()

        self.send_goal_future = self.navigate_to_pose_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        
    def status_callback(self):
        active_nodes = self.get_node_names()
        for task_node in self.task_nodes:
            if task_node not in active_nodes:
                self.STATE = "NAV_TO_DOCK"
                self.task_node = task_node

        for fallback_node in self.fallback_nodes:
            if fallback_node not in active_nodes:
                self.STATE = "PANIC"
                self.fallback_node = fallback_node
        
        if self.PREV_STATE != self.STATE:

            self.state_machine()
    
    def state_machine(self):

        
        self.PREV_STATE = self.STATE

        if self.STATE == "NAV_TO_DOCK":
            print("Entering Nav Stack Fallback")
            print(f"Node {self.task_node} not found.")
            self.goal_pose = self.init_pose
            self.goal_pose.header.frame_id = "map"

            if self.pose_goal_status != GoalStatus.STATUS_EXECUTING:
                print("Returning to dock...")
                self.send_navigate_to_pose()
        elif self.STATE == "DOCK":
            self.send_dock()
            self.STATE = "IDLE"
        elif self.STATE == "PANIC":
            print(f"Node {self.fallback_node} not found.")
            print("Fallback is down. Aborting task!")
        elif self.STATE == "DO_TASK":
            print("Doing task...")
        else: 
            print("Idling...")


def main(args=None):
    rclpy.init()

    # Choose nodes that you don't want to die. If they do die, then the fallback will be initiated 
    # task_nodes = ["pd_controller", "low_level_policy"]
    task_nodes = []
    fallback_nodes = ["bt_navigator", "amcl", "rplidar_node"]
    orchestrator_node = Orchestrator(task_nodes, fallback_nodes)

    rclpy.spin(orchestrator_node)
    orchestrator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

