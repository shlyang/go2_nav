#!/usr/bin/bash

# 创建新的 tmux 会话但不立即附加到会话
tmux new-session -d -s tmp_test

# 第一个窗口 (默认窗口, 索引 0)
# 分割成四个窗格
tmux split-window -h
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v

# 选择第一个窗格
tmux select-pane -t 0
# 在第一个窗格中执行命令
tmux send-keys '1' C-m
tmux send-keys 'roscore' C-m

tmux select-pane -t 1
# 在第一个窗格中执行命令
tmux send-keys '2' C-m
tmux send-keys 'source /opt/ros/noetic/setup.bash' C-m
tmux send-keys 'ros2 run ros1_bridge dynamic_bridge --bridge-all-topics' C-m

tmux select-pane -t 2
# 在第一个窗格中执行命令
tmux send-keys '2' C-m
tmux send-keys 'cd /unitree/module/graph_pid_ws' C-m
tmux send-keys './0_unitree_slam.sh' C-m

tmux select-pane -t 3
# 在第一个窗格中执行命令
tmux send-keys '2' C-m
tmux send-keys 'cd /unitree/lib/unitree_slam/build' C-m
tmux send-keys './demo_xt16 eth0' C-m

# 创建第二个窗口 (索引 1)
tmux new-window
tmux split-window -h
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v

tmux select-pane -t 0
tmux send-keys '1' C-m
tmux send-keys 'cd /home/unitree/navi_ws' C-m
tmux send-keys 'source devel/setup.bash' C-m
tmux send-keys 'roslaunch uni_move_base move_base.launch' C-m

tmux select-pane -t 1
tmux send-keys '1' C-m
tmux send-keys 'cd /home/unitree/navi_ws' C-m
tmux send-keys 'python3 scripts/keyboard_oamove.py' C-m

tmux select-pane -t 2
tmux send-keys '1' C-m
tmux send-keys 'rviz' C-m

tmux select-pane -t 3
tmux send-keys '1' C-m
tmux send-keys 'rostopic echo /cmd_vel' C-m


# 创建第三个窗口 (索引 2)
tmux new-window
tmux split-window -h
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v

tmux select-pane -t 0
tmux send-keys '1' C-m
tmux send-keys 'docker start -ai nanoowl_container' C-m
tmux send-keys 'python3 owl_flask.py' C-m

tmux select-pane -t 1
tmux send-keys '1' C-m
tmux send-keys 'roslaunch realsense2_camera rs_camera.launch' C-m

tmux select-pane -t 2
tmux send-keys '1' C-m
tmux send-keys 'cd /home/unitree/navi_ws' C-m
tmux send-keys 'python3 scripts/test_owl.py' C-m

tmux select-pane -t 3
tmux send-keys '1' C-m
tmux send-keys 'rostopic echo /move_base/status' C-m

# 选择第一个窗口
tmux select-window -t 0

# 附加到会话
tmux attach-session -t tmp_test