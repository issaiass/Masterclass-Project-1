#!/bin/sh

# launch everything
               
xterm -e "source ~/construct_ws/catkin_ws/devel/setup.bash;
          roslaunch box_camera_description warehouse_ur3e_rg2.launch;" &
    
sleep 5

xterm -e "source ~/construct_ws/catkin_ws/devel/setup.bash;
          roslaunch box_camera_description camera_spawn.launch x:=4.71 y:=-4.279791 z:=0.02 yaw:=0.0;" &
        
sleep 5
          
xterm -e "source ~/construct_ws/catkin_ws/devel/setup.bash;
          roslaunch box_camera_description static_transform.launch" &

sleep 5
  
xterm -e "source ~/construct_ws/catkin_ws/devel/setup.bash;
          cd src/project_object_detection/rviz_config;
          rosrun rviz rviz -d od_markers.rviz;" &

sleep 5

xterm -e "source ~/construct_ws/catkin_ws/devel/setup.bash;
          cd src/project_object_detection;
          roslaunch project_object_detection surface_detection_simple.launch;" &

xterm -e "source ~/construct_ws/catkin_ws/devel/setup.bash;
          rostopic echo /graspable_object_pose;" &

sleep 15

xterm -e "source ~/construct_ws/catkin_ws/devel/setup.bash;
          cd src/project_object_detection;
          roslaunch project_object_detection publish_object_position.launch;" &
          

#sleep 5
          
#xterm -e "source ~/construct_ws/catkin_ws/devel/setup.bash;
#          rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller;" &          
