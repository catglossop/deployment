
# Create a new tmux session
session_name="irobot_create3_nav_fallback_$(date +%s)"
tmux new-session -d -s $session_name

# Split the window into three panes
tmux selectp -t 0    # select the first (0) pane
tmux splitw -v -p 50 # split it into two halves
tmux selectp -t 0    # go back to the first pane
tmux splitw -h -p 50 # split it into two halves
tmux selectp -t 2   # go to second pane 
tmux splitw -h -p 50 # split it into two halves
tmux selectp -t 2
# tmux splitw -v -p 50 
# tmux selectp -t 4
# tmux splitw -v -p 50 
# tmux selectp -t 5
# tmux splitw -h -p 50 



# Launch sensors and robot description
tmux select-pane -t 0
tmux send-keys "source ~/create_ws/install/setup.bash" Enter
tmux send-keys "ros2 launch foxglove_bridge foxglove_bridge_launch.xml" Enter

# Launch sensors and robot description
tmux select-pane -t 1
tmux send-keys "source ~/create_ws/install/setup.bash" Enter
tmux send-keys "ros2 service call /reset_pose irobot_create_msgs/ResetPose" Enter
tmux send-keys "ros2 launch deployment robot_launch.py" Enter

# Run the navigation stack
tmux select-pane -t 2
tmux send-keys "source ~/create_ws/install/setup.bash" Enter
tmux send-keys "ros2 launch nav2_bringup navigation_launch.py params_file:=/home/create/create_ws/src/deployment/config/nav2_params.yaml map:=/home/create/create_ws/src/deployment/maps/rail_loop.yaml" Enter

# Run the navigation stack
tmux select-pane -t 3
tmux send-keys "source ~/create_ws/install/setup.bash" Enter
tmux send-keys "ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/create/create_ws/src/deployment/config/slam_toolbox_localization.yaml" Enter

# # Set up the keys to run the slam node
# tmux select-pane -t 3
# tmux send-keys "source ~/create_ws/install/setup.bash" Enter
# tmux send-keys "ros2 run nav2_map_server map_server --ros-args --params-file /home/create/create_ws/src/deployment/config/map_server_params.yaml" Enter 

# tmux select-pane -t 4 
# tmux send-keys "source ~/create_ws/install/setup.bash" Enter
# tmux send-keys "ros2 lifecycle set /map_server configure" Enter 
# tmux send-keys "ros2 lifecycle set /map_server activate" Enter 

# # Set up the keys to run the slam node
# tmux select-pane -t 5
# tmux send-keys "source ~/create_ws/install/setup.bash" Enter
# tmux send-keys "ros2 run nav2_amcl amcl --ros-args --params-file /home/create/create_ws/src/deployment/config/amcl_params.yaml" Enter 

# tmux select-pane -t 6
# tmux send-keys "source ~/create_ws/install/setup.bash" Enter
# tmux send-keys "ros2 lifecycle set /amcl configure" Enter 
# tmux send-keys "ros2 lifecycle set /amcl activate" Enter 


# Attach to the tmux session
tmux -2 attach-session -t $session_name