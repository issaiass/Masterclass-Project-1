#!/bin/sh

# launch everything
          
xterm -e "source ~/construct_ws/catkin_ws/devel/setup.bash;          
          roslaunch box_camera_description warehouse_ur3e_rg2.launch;" &
          
sleep 15
          
xterm -e "source ~/construct_ws/catkin_ws/devel/setup.bash;
          roslaunch my_ur3e_moveit_config pick_place.launch;" &
