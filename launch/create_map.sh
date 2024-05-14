
# Create a new tmux session
session_name="irobot_create3_$(date +%s)"
tmux new-session -d -s $session_name

# Split the window into three panes
tmux selectp -t 0    # select the first (0) pane
tmux splitw -v -p 50 # split it into two halves
tmux selectp -t 0    # go back to the first pane
tmux splitw -h -p 50 # split it into two halves

tmux selectp -t 0 # go back to first pane 
tmux splitw -v -p 50 # split it into two halves

tmux selectp -t 2 # go back to first pane 
tmux splitw -v -p 50 # split it into two halves

tmux selectp -t 4    # select the new, second (2) pane
tmux splitw -h -p 50 # split it into two halves
tmux selectp -t 0    # go back to the first pane

# Launch sensors and robot description
tmux select-pane -t 0
tmux send-keys "source ~/create_ws/install/setup.bash" Enter
tmux send-keys "ros2 launch foxglove_bridge foxglove_bridge_launch.xml" Enter

# Launch sensors and robot description
tmux select-pane -t 1
tmux send-keys "source ~/create_ws/install/setup.bash" Enter
tmux send-keys "ros2 launch deployment sensors_launch.py" Enter

# Run the teleop_launch.py script in the second pane
tmux select-pane -t 2
tmux send-keys "source ~/create_ws/install/setup.bash" Enter
tmux send-keys "ros2 launch deployment teleop_launch.py" Enter

# Run the navigation stack
tmux select-pane -t 3
tmux send-keys "source ~/create_ws/install/setup.bash" Enter
tmux send-keys "ros2 launch nav2_bringup navigation_launch.py params_file:=/home/create/create_ws/src/deployment/config/nav2_params.yaml" Enter

# Set up the keys to run the slam node
tmux select-pane -t 4
tmux send-keys "source ~/create_ws/install/setup.bash" Enter
tmux send-keys "ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/create/create_ws/src/deployment/config/slam_toolbox_mapping.yaml" 

# Have a pane ready to save the map
tmux select-pane -t 5
tmux send-keys "source ~/create_ws/install/setup.bash" Enter
# tmux send-keys "ros2 run nav2_map_server map_saver_cli -f ../maps/$1" 
tmux send-keys "ros2 service call /slam_toolbox/serialize_map slam_toolbox/SerializePoseGraph ../maps/$1" 


tmux select-pane -t 4

# Attach to the tmux session
tmux -2 attach-session -t $session_name