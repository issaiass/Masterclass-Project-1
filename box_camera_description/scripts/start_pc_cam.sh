#!/bin/sh

# launch everything
          
xterm -e "source ~/construct_ws/catkin_ws/devel/setup.bash;          
          roslaunch perception_test_gazebo start_world.launch;" &
          
sleep 5
          
xterm -e "source ~/construct_ws/catkin_ws/devel/setup.bash;
          roslaunch box_camera_description camera_spawn.launch;" &
          
sleep 5

xterm -e "source ~/construct_ws/catkin_ws/devel/setup.bash;
          rosrun rviz rviz -d ~/construct_ws/catkin_ws/src/box_camera_description/rviz_config/perception.rviz" &
