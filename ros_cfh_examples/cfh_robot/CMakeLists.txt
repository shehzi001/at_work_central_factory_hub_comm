cmake_minimum_required(VERSION 2.8.3)
project(cfh_robot)

find_package(catkin REQUIRED 
  COMPONENTS 
    roscpp
    protobuf_comm
    rockin_msgs
    message_generation
)

find_package(Boost REQUIRED COMPONENTS system filesystem thread)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x -DHAVE_LIBCRYPTO")

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

add_message_files(FILES
  AttentionMessage.msg
  CameraControl.msg
)

generate_messages(
  DEPENDENCIES 
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    roscpp
    protobuf_comm
    rockin_msgs
    message_runtime
)

add_executable(cfh_robot_node
   ros/src/cfh_robot_node.cpp
   ros/src/cfh_robot.cpp
)

target_link_libraries(cfh_robot_node
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)

install(
  TARGETS cfh_robot_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
