cmake_minimum_required(VERSION 3.0.2)
project(move_robot)

find_package(catkin REQUIRED COMPONENTS
# Part 1 - Basic Pub Sub
  rospy 
  geometry_msgs
  sensor_msgs
# Part 2 - Services
  std_msgs
  std_srvs
  message_generation 
# Part 3 - Actions
  actionlib_msgs
  nav_msgs
)

add_service_files(
   FILES
# Part 2 - Services
   GoToLoading.srv
)

# Part 3 - Actions
add_action_files(
  FILES 
  OdomRecord.action
)

generate_messages(
  DEPENDENCIES
# Part 2 - Services
  std_msgs
  std_srvs  
# Part 3 - Actions
  actionlib_msgs
  nav_msgs
)

# Part 1 - Basic Pub Sub
catkin_package(
  CATKIN_DEPENDS
  rospy
)

# Part 1 - Basic Pub Sub
include_directories(
 ${catkin_INCLUDE_DIRS}
 INCLUDE
)