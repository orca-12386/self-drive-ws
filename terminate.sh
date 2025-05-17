#!/bin/bash

# Kill tmux session named ros_setup if it exists
tmux has-session -t ros_setup 2>/dev/null
if [ $? -eq 0 ]; then
  echo "Killing tmux session 'ros_setup'..."
  tmux kill-session -t ros_setup
else
  echo "tmux session 'ros_setup' does not exist."
fi

# Stop all running Docker containers
running_containers=$(docker ps -q)
if [ -n "$running_containers" ]; then
  echo "Stopping all running Docker containers..."
  docker stop $running_containers
else
  echo "No running Docker containers to stop."
fi
