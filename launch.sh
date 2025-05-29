#!/bin/bash
# Configuration - easily modify which commands to run
# Format: "command_name|command_to_run|container_or_local"
COMMANDS=(
    # Local commands (outside container)
    "ros1_bridge|source install/setup.bash && ros2 run ros1_bridge dynamic_bridge --bridge-all-topics|local"
    "launcher|sleep 4 && source install/setup.bash && ros2 launch launcher main.launch.py|local"
    "teleop_control|sleep 6 && source install/setup.bash && ./steve_compy_ros2|local"
    # Container commands (inside Docker)
    # "roscore|source /dvolume/devel/setup.bash && roscore|container"
    "zed_wrapper|sleep 2 && source /dvolume/devel/setup.bash && roslaunch zed_wrapper zed_no_tf.launch|container"
    "move_base|sleep 8 && source /dvolume/devel/setup.bash && roslaunch navigation move_base.launch|container"
)

# Set session name
SESSION=ros_setup

# Kill existing session if it exists
tmux kill-session -t $SESSION 2>/dev/null

# Start a new tmux session
tmux new-session -d -s $SESSION

# Setup local environment first
local_count=0
tmux rename-window -t $SESSION:0 "local"

for cmd in "${COMMANDS[@]}"; do
    IFS='|' read -r name command environment <<< "$cmd"
    if [ "$environment" == "local" ]; then
        if [ $local_count -eq 0 ]; then
            # First local command goes to the first pane
            tmux rename-window -t $SESSION:0 "$name"
            tmux send-keys -t $SESSION:0 "$command" C-m
        else
            # Create additional panes as needed - using horizontal split for better visibility
            tmux split-window -v -t $SESSION:0
            tmux select-layout -t $SESSION:0 even-vertical
            tmux send-keys -t $SESSION:0 "$command" C-m
            # Add name as pane title
            tmux select-pane -T "$name"
        fi
        ((local_count++))
    fi
done

# Create a container init script that will automatically start commands in tmux
CONTAINER_INIT_SCRIPT="$(pwd)/ros1-bridge/container_init.sh"
cat > "$CONTAINER_INIT_SCRIPT" << 'EOF'
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
EOF

# Make the container init script executable
chmod +x "$CONTAINER_INIT_SCRIPT"

# Build the container commands string to pass as environment variable
CONTAINER_CMDS=""
for cmd in "${COMMANDS[@]}"; do
    IFS='|' read -r name command environment <<< "$cmd"
    if [ "$environment" == "container" ]; then
        # Add to container commands with name
        if [ -z "$CONTAINER_CMDS" ]; then
            CONTAINER_CMDS="${name}|${command}"
        else
            CONTAINER_CMDS="${CONTAINER_CMDS};${name}|${command}"
        fi
    fi
done

# Create Docker run command
tmux new-window -t $SESSION:1 -n "container"
tmux send-keys -t $SESSION:1 "cd ros1-bridge && docker run -it \\
    --env=\"QT_X11_NO_MITSHM=1\" \\
    --volume=\"\$(pwd):/dvolume\" \\
    --privileged \\
    --device=/dev/ttyACM0 \\
    --device=/dev \\
    --gpus device=0 \\
    -e DISPLAY=:0 \\
    -v /tmp/.X11-unix:/tmp/.X11-unix \\
    --network host \\
    -e ROS_MASTER_URI=http://localhost:11311 \\
    -e \"NVIDIA_DRIVER_CAPABILITIES=all\" \\
    -e \"NVIDIA_VISIBLE_DEVICES=all\" \\
    -e \"CONTAINER_COMMANDS=$CONTAINER_CMDS\" \\
    ros:noetic \\
    /dvolume/container_init.sh" C-m

echo ""
echo "ROS Setup Complete!"
echo "----------------------"
echo "1. Local commands are running in tmux session window 0"
echo "2. Docker container is starting with all commands automatically in window 1"
echo ""
echo "To interact with the sessions:"
echo "  - Use 'tmux attach -t $SESSION' to connect to the session"
echo "  - Switch between windows with Ctrl+B n (next) or Ctrl+B p (previous)"
echo "  - Navigate between panes with Ctrl+B arrow keys"
echo "  - Detach with Ctrl+B d"
echo ""

# Attach to the tmux session
tmux select-window -t $SESSION:0
tmux attach -t $SESSION
