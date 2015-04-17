#!/bin/bash

SESSION=object_learning

# Set this variable in order to have a development workspace sourced, surplus/instead of the .bashrc one
DEVELOPMENT_WS=/home/strands/g4s_ws/devel/setup.bash
_SRC_ENV="tmux send-keys source Space $DEVELOPMENT_WS C-m "

message () { tmux send-keys " #   ===  $1  ===" C-m;  };

split_4 () { tmux split-window -v; tmux select-pane -t 1; tmux split-window -h; tmux select-pane -t 0; tmux split-window -h; tmux select-pane -t 0; };
split_3 () { tmux split-window -v; tmux select-pane -t 1; tmux split-window -h; tmux select-pane -t 0; };
pane () { tmux select-pane -t $1; [ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`; tmux send-keys "clear" C-m; };
cmd () { tmux send-keys "$1"; };
window () { tmux select-window -t $SESSION:$1;  };

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux rename-window -t $SESSION:0 'metric_maps'
tmux new-window -t $SESSION:1 -n 'vision'
tmux new-window -t $SESSION:2 -n 'motion'

window 0
  split_3
    pane 0
      message "Semantic map launch"
      cmd "roslaunch semantic_map_launcher semantic_map.launch "
    pane 1
      message "Metric Map PTU Sweep"
      cmd "roslaunch scitos_ptu ptu_action_server_metric_map.launch"
    pane 2
      messgae "Metric Map Spare"


# Set default window
window 1
  split_4
    pane 0
      message "Camera Tracker"
      cmd "rosrun camera_tracker camera_tracker_service _camera_topic:=/head_xtion/depth_registered"
    pane 1
      message "Object Learning service"
      cmd "rosrun dynamic_object_learning do_learning_service"
    pane 2
      message "Recognition service"
      cmd "roslaunch singleview_object_recognizer recognition_service.launch cg_size:=5"

# Attach to session
window 2
  split_4
    pane 0
      message "Action server"
      cmd "rosrun learn_objects_action server.py"
    pane 1
      message "Trajectory generator"
      cmd "rosrun object_view_generator view_points_service.py"
    pane 2
      message "Waypoint semantic map"
      cmd "rosrun semantic_map_to_2d semantic_map_2d_server"
    pane 3
      message "Object learning spare"
      cmd ""

tmux -2 attach-session -t $SESSION
