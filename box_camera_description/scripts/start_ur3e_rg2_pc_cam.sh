#!/bin/sh

# launch everything
               
xterm -e "source ~/construct_ws/catkin_ws/devel/setup.bash;
          roslaunch box_camera_description warehouse_ur3e_rg2.launch;" &
    
sleep 5

xterm -e "source ~/construct_ws/catkin_ws/devel/setup.bash;
          roslaunch box_camera_description camera_spawn.launch x:=4.71 y:=-4.279791 z:=0.02 yaw:=0.0;" &
        
sleep 5
  
xterm -e "source ~/construct_ws/catkin_ws/devel/setup.bash;
          cd src/box_camera_description/rviz_config;
          rosrun rviz rviz -d ur3e_rg2_pc_cam.rviz;" &

sleep 5

xterm -e "source ~/construct_ws/catkin_ws/devel/setup.bash;
          rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller;" &
          
sleep 5
          
xterm -e "source ~/construct_ws/catkin_ws/devel/setup.bash;
          roslaunch box_camera_description static_transform.launch" &
          
