import rclpy 
import numpy as np
import math
import threading
from rclpy.node import Node 
from sensor_msgs.msg import BatteryState, Joy
from std_msgs.msg import String, Bool
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Twist, PoseStamped, TransformStamped, Quaternion
from irobot_create_msgs.msg import HazardDetectionVector, HazardDetection, Dock
from irobot_create_msgs.action import Undock, DockServo
from nav2_msgs.action import NavigateToPose

from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer


class TaskSwitcher(Node):

    def __init__(self):
        super().__init__('task_switcher')
        self.current_task = "idle"
        self.prev_task = None

        """
        Needs to switch through main tasks: 
            1. Learning task (need to make some kind of config for this)
            2. Reset task (when encountering bumper event/hazard detection that cannot be avoided)
               - Requires interactions with fallback stack 
            3. Teleop task (on teleop button press)
            4. When battery gets too low (< 15%) head back to dock 
        """
        self.init_pose = PoseStamped()
        self.goal_pose = self.init_pose
        self.goal_pose.header.frame_id = "odom"
        self.pose_goal_status = GoalStatus.STATUS_UNKNOWN
        self.return_to_task = True 

        ## SUBSCRIBERS 
        self.irobot_qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT, 
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.VOLATILE,
                depth=1
        )
        # Subscribe to battery state
        self.battery_msg = BatteryState()
        self.battery_sub = self.create_subscription(
            BatteryState,
            "/battery_state", 
            self.battery_callback, 
            self.irobot_qos_profile)
        # Subscribe to hazard detections
        self.hazard_msg = HazardDetectionVector()
        self.hazard_sub = self.create_subscription(
            HazardDetectionVector, 
            "/hazard_detection", 
            self.hazard_callback, 
            self.irobot_qos_profile)
        # Subscribe to teleop msgs
        self.joy_msg = Joy()
        self.joy_sub = self.create_subscription(
            Joy, 
            "/joy",
            self.joy_callback,
            1)
        # Subscribe to dock msgs
        self.dock_msg = Dock()
        self.dock_sub = self.create_subscription(
            Dock, 
            "/dock",
            self.dock_callback,
            self.irobot_qos_profile)

        ## VEL SUBSCRIBERS
        # self.task_vel_msg = Twist()
        # self.task_vel_sub = self.create_subscription(
        #     Twist, 
        #     "/task/cmd_vel",
        #     self.task_vel_callback,
        #     1)
        # self.teleop_vel_msg = Twist()
        # self.teleop_vel_sub = self.create_subscription(
        #     Twist, 
        #     "/teleop/cmd_vel",
        #     self.teleop_vel_callback,
        #     1)
        # self.nav_vel_msg = Twist()
        # self.nav_vel_sub = self.create_subscription(
        #     Twist, 
        #     "/nav2/cmd_vel",
        #     self.nav_vel_callback,
        #     1)

        ## TF LISTENER 
        self.tf_buffer = Buffer()
        self.reset_frame = TransformStamped()
        self.reset_listener = TransformListener(self.tf_buffer, self)

        ## PUBLISHERS 
        self.vel_msg = Twist()
        self.vel_pub = self.create_publisher(
            Twist, 
            "/cmd_vel",
            1
        )
        self.fallback_pub = self.create_publisher(
            Pose, 
            "/goal_pose",
            1
        )
        self.dock_lock_msg = Bool()
        self.dock_lock_msg.data = True
        self.dock_lock_pub = self.create_publisher(
            Bool, 
            "/dock_lock", 
            1
        )
        ## BROADCASTER 
        self.src_frame = "odom"
        self.target_frame = "reset"
        self.reset_broadcaster = TransformBroadcaster(self)

        ## TIMERS 
        self.update_period = 0.01
        self.state_machine_timer = self.create_timer(self.update_period, self.state_machine_callback)
        self.tf_timer = self.create_timer(0.5, self.tf_timer_callback)
        self.dock_lock = self.create_timer(0.01, self.dock_lock_callback)

        ## ACTION CLIENTS
        self.undock_action_client = ActionClient(self, Undock, 'undock')
        self.dock_action_client = ActionClient(self, DockServo, 'dock')
        self.navigate_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

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
        if self.dock_msg.dock_visible:
            self.current_task = "dock"
            self.return_to_task = False
        if self.return_to_task == True:
            self.current_task = "do_task"

    def send_navigate_to_pose(self):
        goal_msg = NavigateToPose.Goal()
        self.pose_goal_status = GoalStatus.STATUS_EXECUTING

        goal_msg.pose = self.goal_pose

        self.navigate_to_pose_client.wait_for_server()

        self.send_goal_future = self.navigate_to_pose_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def battery_callback(self, battery_msg): 

        self.battery_msg = battery_msg
        print("Battery: ", round(self.battery_msg.percentage*100, 2))

        if self.battery_msg.percentage < 0.15: 
            self.prev_task = self.current_task
            if self.dock_msg.dock_visible:
                self.current_task = "dock"
            elif not self.dock_msg.is_docked: 
                self.current_task = "nav_to_dock"
            else:
                self.current_task = "idle"
    
    def hazard_callback(self, hazard_msg):

        self.hazard_msg = hazard_msg

        for detection in self.hazard_msg.detections: 
            if detection.type in [HazardDetection.BUMP, HazardDetection.CLIFF, HazardDetection.STALL]:
                if not self.dock_msg.is_docked and not self.dock_msg.dock_visible and self.current_task != "dock" and self.current_task != "idle":
                    self.prev_task = self.current_task
                    self.current_task = "reset"
    
    def joy_callback(self, joy_msg):
        self.joy_msg = joy_msg

        if self.joy_msg.buttons[5] == 1: 
            self.dock_lock_msg.data = False 
            self.prev_task = self.current_task
            self.current_task = "teleop"
        elif self.joy_msg.buttons[0] == 1: # A button
            self.dock_lock_msg.data = True 
            self.prev_task = self.current_task
            self.current_task = "idle"
        elif self.joy_msg.buttons[1] == 1: # B button
            self.dock_lock_msg.data = False 
            self.prev_task = self.current_task
            if self.dock_msg.dock_visible: 
                self.current_task = "dock"
            elif not self.dock_msg.is_docked:
                self.current_task = "nav_to_dock"

        elif self.joy_msg.buttons[2] == 1: # X button
            self.prev_task = self.current_task
            self.dock_lock_msg.data = False
            self.current_task = "undock"


        if self.current_task == "teleop" and self.joy_msg.buttons[5] == 0: 
            self.prev_task = self.current_task 
            self.current_task = "do_task"

    def dock_callback(self, dock_msg):

        self.dock_msg = dock_msg
    
    def dock_lock_callback(self):
        self.dock_lock_pub.publish(self.dock_lock_msg)
    
    def task_vel_callback(self, vel_msg): 

        self.task_vel_msg = vel_msg

    def teleop_vel_callback(self, vel_msg): 

        self.teleop_vel_msg = vel_msg 
    
    def nav_vel_callback(self, vel_msg):

        self.nav_vel_msg = vel_msg
    
    def tf_timer_callback(self):

        self.broadcast_src_frame = "odom"
        self.broadcast_target_frame = "reset"

        self.listen_src_frame = "odom"
        self.listen_target_frame = "base_link"

        if self.current_task != "reset":
            try: 
                self.reset_frame = self.tf_buffer.lookup_transform(
                    self.listen_src_frame,
                    self.listen_target_frame, 
                    rclpy.time.Time())
            except TransformException as ex: 
                self.get_logger().info(
                    f'Could not transform {self.listen_src_frame} to {self.listen_target_frame}: {ex}')
                return
            self.reset_frame.header.frame_id = self.broadcast_src_frame
            self.reset_frame.child_frame_id = self.broadcast_target_frame
            self.reset_broadcaster.sendTransform(self.reset_frame)
        else:
            self.reset_frame.header.frame_id = self.broadcast_src_frame
            self.reset_frame.child_frame_id = self.broadcast_target_frame
            self.reset_broadcaster.sendTransform(self.reset_frame)

            
    def state_machine_callback(self):
        if self.current_task != self.prev_task: 
            self.state_machine()

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q

    def state_machine(self): 
        print("Current task: ", self.current_task)
        print("Previous task: ", self.prev_task)
        self.prev_task = self.current_task 

        if self.current_task == "idle": 
            self.current_task = "idle"
            print("SETTING DOCK LOCK TRUE")
            self.dock_lock_msg.data = True
        elif self.current_task == "undock": 
            # Send action to undock from dock and start task
            self.send_undock()
            self.current_task = "do_task"

        elif self.current_task == "nav_to_dock": 
            self.goal_pose = self.init_pose
            self.goal_pose.header.frame_id = "odom"

            if self.pose_goal_status != GoalStatus.STATUS_EXECUTING:
                print("Not in dock, returning")
                self.send_navigate_to_pose()

        elif self.current_task == "dock":
            self.send_dock()
            self.current_task = "idle"

        elif self.current_task == "reset": 
            if not self.dock_msg.is_docked and self.return_to_task:
                # self.goal_pose.header.frame_id = "reset"
                # self.goal_pose.pose.position.x = -1.0
                # self.send_navigate_to_pose()
                self.current_task = "do_task"
            else:
                self.current_task = "idle"

        elif self.current_task == "teleop": 
            if self.prev_task == "idle" or self.prev_task == "dock":
                self.current_task = "idle"
            else:
                self.current_task == "do_task"

        elif self.current_task == "do_task": 
            self.dock_lock_msg.data = False
            # DO TASK UNTIL INTERRUPTED 
            self.current_task = "do_task"

        else:         
            self.current_task = "do_task"

def main(args=None):
    rclpy.init()

    task_switcher_node = TaskSwitcher()

    rclpy.spin(task_switcher_node)
    task_switcher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        

        
        

        



            


        




