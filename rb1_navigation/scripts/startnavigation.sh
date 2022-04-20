#!/bin/sh

# launch everything
xterm -e "source ~/construct_ws/catkin_ws/devel/setup.bash;
          roslaunch rb1_navigation main.launch" &
