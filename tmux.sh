#!/bin/bash

# Start a new tmux session
tmux new-session -d -s robot

tmux split-window -h
tmux select-pane -t 0

tmux split-window -v

tmux split-window -h

# Send roslaunch command to the top left pane
tmux select-pane -t 0
tmux send-keys 'cd catkin_ws && catkin build && source devel/setup.bash && roslaunch servo servo.launch' C-m

# Send rostopic echo commands to the bottom left panes
tmux select-pane -t 1
tmux send-keys 'cd catkin_ws/devel' C-m
tmux send-keys 'source setup.bash && rostopic echo /servo_positions'

tmux select-pane -t 2
tmux send-keys 'cd catkin_ws/devel' C-m
tmux send-keys 'source setup.bash && rostopic echo /servo_commands'

tmux select-pane -t 3
tmux send-keys 'cd catkin_ws/devel' C-m
tmux send-keys 'source setup.bash'

# Set the default pane
tmux select-pane -t 1

# Attach to the tmux session
tmux attach-session -t robot