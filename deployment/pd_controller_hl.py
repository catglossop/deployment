import numpy as np
import yaml
from typing import Tuple
import atexit
from PIL import Image as PILImage
import io

# ROS
from topic_names import (WAYPOINT_TOPIC, 
			 			REACHED_GOAL_TOPIC)
import rclpy
from rclpy.node import Node
from ros_data import ROSData
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Bool

from deployment.utils import msg_to_pil, transform_images

CONFIG_PATH = "/home/create/create_ws/src/deployment/config/robot.yaml"

class LifelongLearningPDController(Node): 

    def __init__(self, 
                 config_path,
                 server_ip, 
                 ):
        super().__init__('pd_controller')
        self.load_config(config_path)

        self.vel_msg = Twist()
        self.waypoint = ROSData(self.WAYPOINT_TIMEOUT, name="waypoint")
        self.reached_goal = False
        self.reverse_mode = False
 
        self.waypoint_sub = self.create_subscription(
            Float32MultiArray,
            WAYPOINT_TOPIC,
            self.waypoint_callback,
            1)
        self.reached_goal_sub = self.create_subscription(
            Bool, 
            REACHED_GOAL_TOPIC, 
            self.reached_goal_callback, 
            1)
        
        self.reset_sub = self.create_subscription(
            Bool, 
            "/reset",
            self.task_status_callback,
            1)
        # self.obs_msg = Image()
        # self.obs = None 
        # self.obs_sub = self.create_subscription(
        #     Image, 
        #     "/front/image_raw",
        #     self.obs_callback,
        #     1)

        self.vel_out = self.create_publisher(
            Twist, 
            self.VEL_TOPIC, 
            1)
        self.timer_period = 0.01  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
    
    def load_config(self, config_path):
        with open(config_path, "r") as f:
            robot_config = yaml.safe_load(f)
        self.MAX_V = robot_config["max_v"]
        self.MAX_W = robot_config["max_w"]
        self.VEL_TOPIC = "/task_vel" # CHANGE BACK --> robot_config["vel_navi_topic"]
        self.DT = 1/robot_config["frame_rate"]
        self.RATE = 4
        self.EPS = 1e-8
        self.WAYPOINT_TIMEOUT = 1 # seconds # TODO: tune this

    def load_model_from_config(self, model_paths_config):
        # Load configs
        with open(model_paths_config, "r") as f:
            model_paths = yaml.safe_load(f)

        model_config_path = model_paths["nomad"]["config_path"]
        with open(model_config_path, "r") as f:
            self.model_params = yaml.safe_load(f)

    def clip_angle(self, theta) -> float:
        """Clip angle to [-pi, pi]"""
        theta %= 2 * np.pi
        if -np.pi < theta < np.pi:
            return theta
        return theta - 2 * np.pi
        

    def pd_controller(self, waypoint: np.ndarray) -> Tuple[float]:
        """PD controller for the robot"""
        assert len(waypoint) == 2 or len(waypoint) == 4, "waypoint must be a 2D or 4D vector"
        if len(waypoint) == 2:
            dx, dy = waypoint
        else:
            dx, dy, hx, hy = waypoint
        print("waypoint: ", dx, dy)
        # this controller only uses the predicted heading if dx and dy near zero
        if len(waypoint) == 4 and np.abs(dx) < self.EPS and np.abs(dy) < self.EPS:
            v = 0
            w = self.clip_angle(np.arctan2(hy, hx))/self.DT		
        elif np.abs(dx) < self.EPS:
            v =  0
            w = np.sign(dy) * np.pi/(2*self.DT)
        else:
            v = dx / self.DT
            w = np.arctan(dy/dx) / self.DT
        v = np.clip(v, 0, self.MAX_V)
        w = np.clip(w, -self.MAX_W, self.MAX_W)
        return v, w
    
    def transform_image_to_string(self, pil_img, image_size, center_crop=False):
        """Transforms a PIL image to a torch tensor."""
        w, h = pil_img.size
        if center_crop: 
            if w > h:
                pil_img = TF.center_crop(pil_img, (h, int(h * self.image_aspect_ratio)))
            else:
                pil_img = TF.center_crop(pil_img, (int(w / self.image_aspect_ratio), w))
        pil_img = pil_img.resize(image_size)
        image_bytes_io = io.BytesIO()
        PILImage.fromarray(np.array(self.obs)).save(image_bytes_io, format = 'JPEG')
        image_bytes = tf.constant(image_bytes_io.getvalue(), dtype = tf.string)
        return image_bytes

    def waypoint_callback(self, waypoint_msg: Float32MultiArray):
        """Callback function for the waypoint subscriber""" 
        # TODO: add config for different kinds of waypoints/goals?
        print("Setting waypoint")
        self.waypoint.set(waypoint_msg.data)
    
    def obs_callback(self, obs_msg: Image):
        """Callback function for the observation subscriber"""
        self.obs = self.transform_image_to_string(msg_to_pil(obs_msg))

    def subgoal_callback(self, msg):
        print("Subgoal received")
        self.subgoal_image = msg_to_pil(msg)
    
    def odom_callback(self, odom_msg: Odometry):
        """Callback function for the odometry subscriber"""
        self.current_yaw = odom_msg.pose.pose.orientation.z
        self.current_pos = np.array([odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y])

    def reached_goal_callback(self, reached_goal_msg: Bool):
        """Callback function for the reached goal subscriber"""
        print("Checking goal reached")
        self.reached_goal = reached_goal_msg.data
    
    def timer_callback(self):
        
        if self.reached_goal:
            self.vel_out.publish(self.vel_msg)
            print("Reached goal! Finding new subgoal...")
            
            return
        elif self.waypoint.is_valid(verbose=True):
            v, w = self.pd_controller(self.waypoint.get())

            if self.reverse_mode: 
                v *= -1
            self.vel_msg.linear.x = v
            self.vel_msg.angular.z = w
            print(f"publishing new vel: {v}, {w}")
        self.vel_out.publish(self.vel_msg)       


def main(args=None):
    rclpy.init()
    print("Registered with master node. Waiting for waypoints...")
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="localhost")
    args = parser.parse_args()
    pd_controller_node = LifelongLearningPDController(CONFIG_PATH, args.ip)
    # pd_controller_node.trainer.recv_network_callback(_update_actor)
    pd_controller_node.trainer.start_async_update(interval=1)
    rclpy.spin(pd_controller_node)
    pd_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()