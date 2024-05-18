import rclpy 
import numpy as np
import math
import threading
from rclpy.node import Node 
from sensor_msgs.msg import BatteryState, Joy, Imu
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
        self.MAX_ATTEMPTS = 3
        self.TIMEOUT = 15.0
        self.attempts = 0

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

        # Subscribe to IMU 
        self.imu_msg = Imu()
        self.imu_sub = self.create_subscription(
            Imu, 
            "/imu",
            self.imu_callback,
            self.irobot_qos_profile)
        self.IMU_LIMIT = 6.0

        ## TF LISTENER 
        self.tf_buffer = Buffer()
        self.reset_frame = TransformStamped()
        self.reset_listener = TransformListener(self.tf_buffer, self)

        ## PUBLISHERS 
        self.dock_lock_msg = Bool()
        self.dock_lock_msg.data = True
        self.dock_lock_pub = self.create_publisher(
            Bool, 
            "/dock_lock", 
            1
        )
        self.state_msg = String()
        self.state_pub = self.create_publisher(
            String, 
            "/hierarchical_learning/state", 
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
        self.state_timer = self.create_timer(0.01, self.state_timer_callback)


        ## ACTION CLIENTS
        self.undock_action_client = ActionClient(self, Undock, 'undock')
        self.dock_action_client = ActionClient(self, DockServo, 'dock')
        self.navigate_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    # ACTIONS
    def send_undock(self):
        goal_msg = Undock.Goal()
        
        self.undock_action_client.wait_for_server()

        return self.undock_action_client.send_goal_async(goal_msg)

    def send_dock(self):
        goal_msg = DockServo.Goal()

        self.dock_action_client.wait_for_server()

        return self.dock_action_client.send_goal_async(goal_msg)

    # HANDLING RESET ACTION 
    def reset_goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            print('Goal rejected :(')
            return
        print('Goal accepted :)')
        self.get_result_future = self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_reset_result_callback)

    def reset_cancel_response_callback(self, future):
        print("IN CANCEL RESPONSE")
        self.cancel_handle = future.result()
        print('Cancel accepted :)')
        self.attempts += 1

    def reset_feedback_callback(self, feedback_msg):
        print("TIME: ", feedback_msg.feedback.navigation_time.sec)
        print("NUM RECOVERIES: ", feedback_msg.feedback.number_of_recoveries)
        if self.hazard_msg.detections: 
            if self.hazard_msg.detections[0].type in [HazardDetection.BUMP, HazardDetection.CLIFF, HazardDetection.STALL]:
                self.get_logger().info('Hazard detected: {0}'.format(self.hazard_msg.detections[0].type))
                self.get_logger().info('Stopping current action')
                try:
                    self.cancel_goal_future = self.cancel_handle.cancel_goal_async()
                    self.cancel_goal_future.add_done_callback(self.cancel_response_callback)
                except:
                    pass
                if self.attempts < self.MAX_ATTEMPTS:
                    self.current_task = "reset"
                else:
                    self.current_task = "do_task"
        
        if feedback_msg.feedback.navigation_time.sec >= self.TIMEOUT: 
            self.get_logger().info('Goal timed out')
            self.get_logger().info('Stopping current action')
            self.cancel_goal_future = self.goal_handle.cancel_goal_async()
            self.cancel_goal_future.add_done_callback(self.reset_cancel_response_callback)
            if self.attempts < self.MAX_ATTEMPTS:
                self.current_task = "reset"
            else:
                self.current_task = "do_task"
        
    def get_reset_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result))
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))
        self.attempts = 0
        self.pose_goal_status = status
        if self.return_to_task == True:
            self.current_task = "do_task"

    def send_navigate_to_pose_reset(self):
        goal_msg = NavigateToPose.Goal()
        self.pose_goal_status = GoalStatus.STATUS_EXECUTING

        goal_msg.pose = self.goal_pose

        self.navigate_to_pose_client.wait_for_server()

        self.hazard_msg = HazardDetectionVector()

        self.send_goal_future = self.navigate_to_pose_client.send_goal_async(goal_msg, feedback_callback=self.reset_feedback_callback)
        self.send_goal_future.add_done_callback(self.reset_goal_response_callback)
   

    # HANDLING NAV_TO_DOCK ACTION 
    def dock_goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            print('Goal rejected :(')
            return

        print('Goal accepted :)')

        self.get_result_future = self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_dock_result_callback)

    def dock_cancel_response_callback(self, future):
        print("IN CANCEL RESPONSE")
        self.cancel_handle = future.result()

        print('Cancel accepted :)')
        self.attempts += 1

    def dock_feedback_callback(self, feedback_msg):
        print("TIME: ", feedback_msg.feedback.navigation_time.sec)
        print("NUM RECOVERIES: ", feedback_msg.feedback.number_of_recoveries)
        if self.hazard_msg.detections: 
            if self.hazard_msg.detections[0].type in [HazardDetection.BUMP, HazardDetection.CLIFF, HazardDetection.STALL]:
                self.get_logger().info('Hazard detected: {0}'.format(self.hazard_msg.detections[0].type))
                self.get_logger().info('Stopping current action')
                try:
                    self.cancel_goal_future = self.cancel_handle.cancel_goal_async()
                    self.cancel_goal_future.add_done_callback(self.dock_cancel_response_callback)
                except:
                    pass
                if self.attempts < self.MAX_ATTEMPTS:
                    self.current_task = "reset"
                else:
                    self.current_task = "nav_to_dock"
        

    def get_dock_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result))
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))
        self.attempts = 0
        self.pose_goal_status = status
        if self.dock_msg.dock_visible:
            self.current_task = "dock"
        else:
            self.current_task = "nav_to_dock"
            self.attempts += 1
        if self.attempts > self.MAX_ATTEMPTS: 
            self.current_task = "idle"

    def send_navigate_to_pose_dock(self):
        goal_msg = NavigateToPose.Goal()
        self.pose_goal_status = GoalStatus.STATUS_EXECUTING

        goal_msg.pose = self.goal_pose

        self.navigate_to_pose_client.wait_for_server()

        self.hazard_msg = HazardDetectionVector()

        self.send_goal_future = self.navigate_to_pose_client.send_goal_async(goal_msg, feedback_callback=self.dock_feedback_callback)
        self.send_goal_future.add_done_callback(self.dock_goal_response_callback)

    # CALLBACKS
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
    
    def imu_callback(self, imu_msg):
        self.imu_msg = imu_msg
        imu_x = self.imu_msg.linear_acceleration.x
        imu_y = self.imu_msg.linear_acceleration.y
        imu_z = self.imu_msg.linear_acceleration.z
        if abs(imu_x) > 4.0 or abs(imu_y) > 4.0 or abs(imu_z) > 4.0:
            print("Robot has been bumped: ", imu_x, imu                                                                                                                                                                                                                                                                                                                                                                                                                                              _y, imu_z)
            if not self.dock_msg.is_docked and not self.dock_msg.dock_visible and self.current_task != "dock" and self.current_task != "idle":
                self.prev_task = self.current_task
                self.current_task = "reset"

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
    
    def state_timer_callback(self):
        self.state_msg = self.current_task
        self.state_pub.publish(self.state_msg)

            
    def state_machine_callback(self):
        if self.current_task != self.prev_task: 
            self.state_machine()

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Below should be replaced when porting for ROS 2 Python tf_conversions is done.
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
                self.send_navigate_to_pose_dock()

        elif self.current_task == "dock":
            self.send_dock()
            self.current_task = "idle"

        elif self.current_task == "reset": 
            if not self.dock_msg.is_docked and self.return_to_task:
                self.goal_pose.header.frame_id = "reset"
                self.goal_pose.pose.position.x = -1.0
                self.send_navigate_to_pose_reset()
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
        

        
        

        



            


        




