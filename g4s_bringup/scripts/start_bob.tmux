#!/bin/bash

SESSION=g4s
DATABASE=/data/1f_mongo_140415

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'mongo'
tmux new-window -t $SESSION:2 -n 'robot'
tmux new-window -t $SESSION:3 -n 'cameras'
tmux new-window -t $SESSION:4 -n 'ui'
tmux new-window -t $SESSION:5 -n 'navigation'
tmux new-window -t $SESSION:6 -n 'ppl_perception'
tmux new-window -t $SESSION:7 -n 'executive'
tmux new-window -t $SESSION:8 -n 'logging'

tmux select-window -t $SESSION:0
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "roscore" C-m
tmux resize-pane -U 20
tmux select-pane -t 1
tmux send-keys "htop" C-m

tmux select-window -t $SESSION:1
tmux split-window -v
tmux select-pane -t 0
tmux send-keys " clear" C-m
tmux send-keys "roslaunch strands_bringup strands_core.launch db_path:=$DATABASE"
tmux select-pane -t 1
tmux split-window -h
tmux select-pane -t 1
tmux send-keys " clear" C-m
tmux send-keys "while true; do date;echo ;df -h /data; sleep 36000; echo "-----"; done" C-m
tmux select-pane -t 2
tmux send-keys " clear" C-m
tmux send-keys "## DATABASE REPLICATION ###"

tmux select-window -t $SESSION:2
tmux split-window -h
tmux select-pane -t 0
tmux send-keys " clear" C-m
tmux send-keys "roslaunch strands_bringup strands_robot.launch"
tmux select-pane -t 1
tmux split-window -v
tmux select-pane -t 1
tmux send-keys " clear" C-m
tmux send-keys "# FAILSAFE SHUTDOWN / EMAIL SCRIPT"
tmux select-pane -t 2
tmux send-keys " clear" C-m
tmux send-keys "# Battery level monitor"

tmux select-window -t $SESSION:3
tmux split-window -v
tmux select-pane -t 0
tmux send-keys " clear" C-m
tmux send-keys "roslaunch strands_bringup strands_cameras.launch head_camera:=false chest_camera:=true"
tmux select-pane -t 1
tmux send-keys " clear" C-m
tmux send-keys "ssh strands@bobl" C-m
tmux send-keys "roslaunch openni_wrapper main.launch camera:=head_xtion"

tmux select-window -t $SESSION:4
tmux split-window -h
tmux select-pane -t 0
tmux send-keys " clear" C-m
tmux send-keys "roslaunch strands_bringup strands_ui.launch"
tmux select-pane -t 1
tmux send-keys " clear" C-m
tmux send-keys "### ROBBLOG ###"

tmux select-window -t $SESSION:5
tmux split-window -v
tmux select-pane -t 0
tmux send-keys " clear" C-m
tmux send-keys "roslaunch strands_bringup strands_navigation map:=!!!!! topological_map:=!!!!!"
tmux select-pane -t 1
tmux split-window -h
tmux select-pane -t 1
tmux send-keys " clear" C-m
tmux send-keys "rostopic echo /current_node" C-m
tmux select-pane -t 2
tmux send-keys " clear" C-m
tmux send-keys "## some more naviation stuff##"

tmux select-window -t $SESSION:6
tmux send-keys " clear" C-m
tmux send-keys "roslaunch perception_people_launch people_tracker_robot.launch"

tmux select-window -t $SESSION:7
tmux split-window -h
tmux select-pane -t 1
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 0
tmux send-keys " clear" C-m
tmux send-keys "## executive ###"
tmux select-pane -t 1
tmux send-keys " clear" C-m
tmux send-keys "## routine ###"
tmux select-pane -t 2
tmux send-keys " clear" C-m
tmux send-keys "## tasks list ###"
tmux select-pane -t 3
tmux send-keys " clear" C-m
tmux send-keys "## something useful ###"

tmux select-window -t $SESSION:8
tmux send-keys " clear" C-m
tmux send-keys "## Logging: rosout, etc ##"

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION

