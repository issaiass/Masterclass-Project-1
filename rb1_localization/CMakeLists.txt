cmake_minimum_required(VERSION 3.0.2)
project(rb1_localization)

find_package(catkin REQUIRED COMPONENTS
  rospy
  amcl
  std_msgs
  message_generation
)

add_service_files(
   FILES
   SavePOI.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rb1_localization
  CATKIN_DEPENDS amcl rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
