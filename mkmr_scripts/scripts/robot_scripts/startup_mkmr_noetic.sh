#!/bin/bash
tmux attach -t mkmr
sleep 1
tmux new-session -d -s mkmr 'source /opt/ros/noetic/setup.bash && source ~/dep_ws/devel/setup.bash && roscore;bash -i'
sleep 1
tmux split-window -h 'tmux set -g mouse on;bash -i'
tmux split-window -v 'source /opt/ros/noetic/setup.bash && source ~/dep_ws/devel/setup.bash && source ~/mkmr_ws/devel/setup.bash && roslaunch mkmr_bringup bringup.launch;bash -i'
tmux select-pane -L
tmux split-window -v 'source /opt/ros/noetic/setup.bash && source ~/dep_ws/devel/setup.bash && source ~/mkmr_ws/devel/setup.bash && echo ros1..;bash -i'
sleep 1
tmux select-pane -U
tmux split-window -v 'source /opt/ros/noetic/setup.bash && source ~/dep_ws/devel/setup.bash && source ~/mkmr_ws/devel/setup.bash && roslaunch mkmr_gazebo mkmr_gazebo.launch;bash -i'
tmux select-pane -R
tmux split-window -v 'source /opt/ros/noetic/setup.bash && source ~/dep_ws/devel/setup.bash && source ~/mkmr_ws/devel/setup.bash && roslaunch smr_viz robot_rviz.launch robot_id:=mkmr0;bash -i'
tmux split-window -h 'echo ..;bash -i'
