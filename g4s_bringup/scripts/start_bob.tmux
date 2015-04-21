#!/bin/bash

SESSION=g4s

DATABASE=/data/1f_mongo_140415
MAP=/opt/maps/1f_pre_deployment/cs_1f_20140724-cropped.yaml
NO_GO_MAP=/opt/maps/1f_pre_deployment/cs_1f_20140724-cropped.yaml
TOPOLOGICAL_MAP=1f_pre_deployment

# Set this variable in order to have a development workspace sourced, surplus/instead of the .bashrc one
DEVELOPMENT_WS=/home/strands/1f_ws/devel/setup.bash
_SRC_ENV="tmux send-keys source Space $DEVELOPMENT_WS C-m "

message () { tmux send-keys " #   ===  $1  ===" C-m;  };

split_4 () { tmux split-window -v; tmux select-pane -t 1; tmux split-window -h; tmux select-pane -t 0; tmux split-window -h; tmux select-pane -t 0; };

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
tmux new-window -t $SESSION:9 -n 'object_learning'
tmux new-window -t $SESSION:10 -n 'offline_learning'
tmux new-window -t $SESSION:11 -n 'novelty_detection'

tmux select-window -t $SESSION:0
tmux split-window -v
tmux select-pane -t 0
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "roscore" C-m
tmux resize-pane -U 20
tmux select-pane -t 1
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "htop" C-m

tmux select-window -t $SESSION:1
tmux split-window -v
tmux select-pane -t 0
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys " clear" C-m
message "MongoDB message_store"
tmux send-keys "roslaunch strands_bringup strands_core.launch db_path:=$DATABASE"
tmux select-pane -t 1
tmux split-window -h
tmux select-pane -t 1
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys " clear" C-m
message "Disk usage monitor"
tmux send-keys "while true; do date;echo ;df -h /data; sleep 36000; echo "-----"; done" C-m
tmux select-pane -t 2
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys " clear" C-m
message "Database replication"
#tmux send-keys "## DATABASE REPLICATION ###"

tmux select-window -t $SESSION:2
tmux split-window -h
tmux select-pane -t 0
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys " clear" C-m
message "Robot drivers"
tmux send-keys "roslaunch strands_bringup strands_robot.launch"
tmux select-pane -t 1
tmux split-window -v
tmux select-pane -t 1
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys " clear" C-m
message "Failsafe shutdown script"
#tmux send-keys "# FAILSAFE SHUTDOWN / EMAIL SCRIPT"
tmux select-pane -t 2
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys " clear" C-m
message "Battery level monitor"
#tmux send-keys "# Battery level monitor"

tmux select-window -t $SESSION:3
tmux split-window -v
tmux select-pane -t 0
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys " clear" C-m
message "Chest Xtion"
tmux send-keys "roslaunch strands_bringup strands_cameras.launch head_camera:=false chest_camera:=true"
tmux select-pane -t 1
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys " clear" C-m
message "Head Xtion"
tmux send-keys "ssh strands@bobl" C-m
tmux send-keys "roslaunch openni_wrapper main.launch camera:=head_xtion"

tmux select-window -t $SESSION:4
tmux split-window -h
tmux select-pane -t 0
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys " clear" C-m
message "STRANDS UI"
tmux send-keys "roslaunch strands_bringup strands_ui.launch"
tmux select-pane -t 1
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys " clear" C-m
message "Roblog server"
#tmux send-keys "### ROBBLOG ###"

tmux select-window -t $SESSION:5
tmux split-window -v
tmux select-pane -t 0
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys " clear" C-m
message "Navigation (monitored & topological & MDP)"
tmux send-keys "roslaunch strands_bringup strands_navigation.launch map:=$MAP no_go_map:=$NO_GO_MAP topological_map:=$TOPOLOGICAL_MAP with_head_xtion:=true with_chest_xtion:=true"
tmux select-pane -t 1
tmux split-window -h
tmux select-pane -t 1
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys " clear" C-m
message "Currrent Waypoint"
tmux send-keys "rostopic echo /current_node" C-m
tmux select-pane -t 2
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys " clear" C-m
message "Spare nav..."
#tmux send-keys "## some more naviation stuff##"

tmux select-window -t $SESSION:6
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux split-window -h
tmux split-window -v
tmux select-pane -t 0
tmux send-keys " clear" C-m
message "Person Tracker"
tmux send-keys "roslaunch perception_people_launch people_tracker_robot.launch"
tmux select-pane -t 1
tmux send-keys " clear" C-m
message "Trajectory Logger"
tmux send-keys "roslaunch human_trajectory trajectory_publisher.py 1.0 1"
tmux select-pane -t 2
tmux send-keys " clear" C-m
message "Robot Pose Logger"
tmux send-keys "rosrun mongodb_log mongodb_log.py /robot_pose"


tmux select-window -t $SESSION:7
tmux split-window -h
tmux select-pane -t 1
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 0
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys " clear" C-m
message "Scheduler & Executor"
tmux send-keys "roslaunch task_executor task-scheduler-mdp.launch topological_map:=$TOPOLOGICAL_MAP"
tmux select-pane -t 1
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys " clear" C-m
message "Routine"
tmux send-keys "rosrun g4s_y2_routine pre_deployment_routine.py"
tmux select-pane -t 2
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys " clear" C-m
message "Schedule Status"
tmux send-keys "rosrun task_executor schedule_status.py"
tmux select-pane -t 3
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys " clear" C-m
message "Spare scheduling terminal..."
#tmux send-keys "## something useful ###" C-m

tmux select-window -t $SESSION:8
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys " clear" C-m
message "ROS-Out logging"
tmux send-keys "## Logging: rosout, etc ##" C-m

tmux select-window -t $SESSION:9
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys " clear" C-m
split_4
message "PTU Frame Follower"
tmux send-keys "rosrun ptu_follow_frame ptu_follow.py "
tmux select-pane 1
message "Static TF Manager"
tmux send-keys "rosrun static_transform_manager static_tf_services.py"
tmux select-pane 2
message "BettyL"
tmux send-keys "ssh bobl" C-m
tmux select-pane 3
message "Object Learning Spare"

tmux select-window -t $SESSION:10
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys " clear" C-m
tmux split-window -h
tmux select-pane -t 0
tmux send-keys " clear" C-m
message "QSRLib Service"
tmux send-keys "rosrun qsr_lib qsrlib_ros_server.py"
tmux select-pane -t 1
tmux send-keys " clear" C-m
message "Offline Learning"
tmux send-keys "rosrun relational_learner OfflineLearning_action.py"

tmux select-window -t $SESSION:11
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
split_4
tmux send-keys " clear" C-m
message "Episode Server"
tmux send-keys "rosrun relational_learner episodes_server.py"
tmux select-pane -t 1
tmux send-keys " clear" C-m
message "Episode Client"
tmux send-keys "rosrun relational_learner episodes_client.py"
tmux select-pane -t 2
tmux send-keys " clear" C-m
message "Novelty Server"
tmux send-keys "rosrun relational_learner novelty_server.py"
tmux select-pane -t 3
tmux send-keys " clear" C-m
message "Novelty Client"
tmux send-keys "rosrun relational_learner novelty_client.py"


# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION

