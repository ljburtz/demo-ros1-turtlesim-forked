#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash
source ~/ros_workspace/devel/setup.bash --extend

# Convenience aliases (add your own to learn from each other!)
echo "alias cc='cd ~/catkin_ws/; catkin build'" >> ~/.bashrc
echo "alias cx='catkin build --no-deps'" >> ~/.bashrc  # usage: cx package_name  <- when you are iterating on a single package and know that no other dependency changed
echo "alias ll='ls -lath'" >> ~/.bashrc
echo "alias kk=clear" >> ~/.bashrc
echo "alias sourceb='source ~/.bashrc'" >> ~/.bashrc
echo "alias jj='jupyter notebook'" >> ~/.bashrc
echo "alias tt='tmux new'" >> ~/.bashrc
echo "alias rc='roscore &'" >> ~/.bashrc
echo "alias ts='rosrun turtlesim turtlesim_node'" >> ~/.bashrc

# Setup for tmux
echo "set-option -g default-command \"exec /bin/bash\"" > ~/.tmux.conf
{
  echo "# if running bash"
  echo "  if [ -n \"$BASH_VERSION\" ]; then"
  echo "    # include .bashrc if it exists"
  echo "    if [ -f \"$HOME/.bashrc\" ]; then"
  echo "    . \"$HOME/.bashrc\""
  echo "    fi"
  echo "fi"
} >> ~/.profile

exec "$@"
