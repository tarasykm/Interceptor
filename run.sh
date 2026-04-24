#!/bin/bash
set -e

WORKSPACE="$(cd "$(dirname "$0")" && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WORKSPACE/install/setup.bash"
SESSION="interceptor"

if [ ! -f "$WS_SETUP" ]; then
    echo "Workspace not built — run 'colcon build' first."
    exit 1
fi

export ROS_DOMAIN_ID=42
SOURCE="source $ROS_SETUP && source $WS_SETUP && export ROS_DOMAIN_ID=42"

tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION" -x 220 -y 50

# Left pane: launch (nodes + RViz2 window)
tmux send-keys -t "$SESSION:0.0" "$SOURCE && ros2 launch interceptor_drone intercept_sim.launch.py" Enter
tmux select-pane -t "$SESSION:0.0" -T "sim"

# Right column: split off for pose monitors
tmux split-window -h -t "$SESSION:0.0" -p 38

# Top-right: lead pose
tmux send-keys -t "$SESSION:0.1" "$SOURCE && sleep 2 && ros2 topic echo /lead/pose" Enter
tmux select-pane -t "$SESSION:0.1" -T "/lead/pose"

# Bottom-right: interceptor pose
tmux split-window -v -t "$SESSION:0.1"
tmux send-keys -t "$SESSION:0.2" "$SOURCE && sleep 2 && ros2 topic echo /interceptor/pose" Enter
tmux select-pane -t "$SESSION:0.2" -T "/interceptor/pose"

# Focus the launch pane
tmux select-pane -t "$SESSION:0.0"

tmux attach-session -t "$SESSION"
