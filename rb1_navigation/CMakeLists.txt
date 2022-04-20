cmake_minimum_required(VERSION 3.0.2)
project(rb1_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)

add_service_files(
   FILES
   GoToPoint.srv
)

generate_messages(
   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
