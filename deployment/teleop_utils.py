import numpy as np
import yaml
from typing import Tuple

# ROS
from .topic_names import (WAYPOINT_TOPIC, 
			 			REACHED_GOAL_TOPIC)
import rclpy
from rclpy.node import Node
from .ros_data import ROSData

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Bool
from sensor_msgs.msg import Joy
from irobot_create_msgs.action import Undock, DockServo
from irobot_create_msgs.msg import Dock 
from rclpy.action import ActionClient

CONFIG_PATH = "../config/robot.yaml"

class TeleopUtils(Node): 

    def __init__(self):
        super().__init__('teleop_utils')

        self.joy_msg = Joy()

        self.joy_sub = self.create_subscription(
            Joy,
            "/joy",
            self.joy_callback,
            1)
 
        self.timer_period = 0.01  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.undock_action_client = ActionClient(self, Undock, 'undock')
        self.dock_action_client = ActionClient(self, DockServo, 'dock')
        self.undock = False 
        self.dock = False

    def send_dock(self):
        goal_msg = DockServo.Goal()

        self.dock_action_client.wait_for_server()

        return self.dock_action_client.send_goal_async(goal_msg)
    
    def send_undock(self):
        goal_msg = Undock.Goal()
        
        self.undock_action_client.wait_for_server()

        return self.undock_action_client.send_goal_async(goal_msg)

    def joy_callback(self, joy_msg: Joy):
        """Callback function for the joy subscriber"""
        self.joy_msg = joy_msg
        if self.joy_msg.buttons[1] == 1 and not self.undock: 
            self.dock = True
        if self.joy_msg.buttons[2] == 1 and not self.dock:
            self.undock = True
    
    def timer_callback(self):

        if self.dock:
            print("Docking...")
            self.send_dock()
            self.dock = False 
        if self.undock:
            print("Undocking...")
            self.send_undock()      
            self.undock = False

def main(args=None):
    rclpy.init(args=args)
    teleop_utils_node = TeleopUtils()
    rclpy.spin(teleop_utils_node)
    teleop_utils_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
	main()