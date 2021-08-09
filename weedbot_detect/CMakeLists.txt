cmake_minimum_required(VERSION 3.0.2)
project(weedbot_detect)

find_package(catkin REQUIRED COMPONENTS
   message_generation
   std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   object.msg
   detection.msg
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES weedbot_detect
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)
