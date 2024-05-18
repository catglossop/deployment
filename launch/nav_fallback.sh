
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
tmux send-keys "ros2 launch nav2_bringup bringup_launch.py params_file:=/home/create/create_ws/src/deployment/config/nav2_params.yaml map:=/home/create/create_ws/src/deployment/maps/rail_loop.yaml" Enter

# Attach to the tmux session
tmux -2 attach-session -t $session_name