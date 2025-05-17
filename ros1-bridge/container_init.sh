#!/bin/bash
# This script will run all container commands in tmux

# Set session name
CONTAINER_SESSION=container_ros

# Start a new tmux session inside container
tmux new-session -d -s $CONTAINER_SESSION

# Counter for panes
container_count=0

# Parse the commands from environment variable
IFS=';' read -ra CONTAINER_CMDS <<< "$CONTAINER_COMMANDS"

for cmd in "${CONTAINER_CMDS[@]}"; do
    IFS='|' read -r name command <<< "$cmd"
    
    if [ $container_count -eq 0 ]; then
        # First command goes to the first pane
        tmux rename-window -t $CONTAINER_SESSION:0 "$name"
        tmux send-keys -t $CONTAINER_SESSION:0 "$command" C-m
    else
        # Create additional panes as needed
        tmux split-window -v -t $CONTAINER_SESSION:0
        tmux select-layout -t $CONTAINER_SESSION:0 even-vertical
        tmux send-keys -t $CONTAINER_SESSION:0 "$command" C-m
        # Add name as pane title
        tmux select-pane -T "$name"
    fi
    ((container_count++))
done

# Attach to the tmux session - this keeps the container running
tmux attach -t $CONTAINER_SESSION
