
# Create a new tmux session
session_name="irobot_create3_task_infra_$(date +%s)"
tmux new-session -d -s $session_name

# Split the window into three panes
tmux selectp -t 0    # go back to the first pane
tmux splitw -h -p 50 # split it into two halves



# Run the task swticher for the fallback
tmux select-pane -t 0
tmux send-keys "source ~/create_ws/install/setup.bash" Enter
tmux send-keys "conda deactivate" Enter
tmux send-keys "python /home/create/create_ws/src/deployment/deployment/task_switcher_mux.py" Enter

# Run the orchestrator for the fallback
tmux select-pane -t 1
tmux send-keys "source ~/create_ws/install/setup.bash" Enter
tmux send-keys "conda deactivate" Enter
tmux send-keys "python /home/create/create_ws/src/deployment/deployment/orchestrator.py" Enter

tmux select-pane -t 0

# Attach to the tmux session
tmux -2 attach-session -t $session_name

