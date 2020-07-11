!/bin/bash
SESSION=core_nav
## Start up
tmux -2 new-session -d -s $SESSION

tmux split-window -v -p 50
tmux split-window -h -p 50
tmux select-pane -t 0
tmux split-window -h -p 50


tmux send-keys -t 0 "sleep 3; roslaunch init_core_nav init_corenav.launch " C-m
tmux send-keys -t 1 "sleep 5; roslaunch core_nav corenav.launch" C-m
tmux send-keys -t 2 "sleep 4; cd ~/Desktop; rosbag play 2019-06-28-17-16-48.bag " C-m
tmux send-keys -t 3 "cd ../../..; source devel/setup.bash" C-m
tmux send-keys -t 4 "rqt_multiplot" C-m

# Attach to session
tmux -2 attach-session -t $SESSION
