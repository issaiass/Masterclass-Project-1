#!/bin/sh

# launch gazebo
xterm -e "source ~/simulation/devel/setup.bash;
          roslaunch rb1_base_gazebo warehouse_rb1.launch" &

sleep 2

xterm -e "source ~/simulation/devel/setup.bash;
          cd ~/catkin_ws;
          catkin_make;" &

sleep 15

# call gmapping
xterm -e "source ~/catkin_ws/devel/setup.bash;
         roslaunch rb1_mapping mapping.launch" &

sleep 2

# call rviz
xterm -e "source ~/catkin_ws/devel/setup.bash;
          roscd rb1_mapping/config
          rosrun rviz rviz -d gmapping.rviz;" &

sleep 2

# call teleoperation
xterm -e "source ~/catkin_ws/devel/setup.bash;
          rosrun rb1_mapping teleop_twist_keyboard.py;" &
