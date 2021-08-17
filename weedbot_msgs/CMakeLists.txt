cmake_minimum_required(VERSION 3.0.2)
project(weedbot_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
)

add_message_files(
   FILES
   Object.msg
   Detection.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   Localize.srv
   PoseListArmCommand.srv
   NamedArmCommand.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES weedbot_msgs
  CATKIN_DEPENDS geometry_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
