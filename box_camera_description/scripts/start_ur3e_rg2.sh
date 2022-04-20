#!/bin/sh

# launch everything
               
xterm -e "source ~/construct_ws/catkin_ws/devel/setup.bash;
          roslaunch box_camera_description warehouse_ur3e_rg2.launch;" &
    
sleep 5

xterm -e "source ~/construct_ws/catkin_ws/devel/setup.bash;
          rosrun rviz rviz -d ~/construct_ws/catkin_ws/src/box_camera_description/rviz_config/ur3e_rg2.rviz" &
