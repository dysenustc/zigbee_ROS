cmake_minimum_required(VERSION 2.8.3)
project(zbserial)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(zbserial_node src/zbserial_node.cpp)

target_link_libraries(zbserial_node
  ${catkin_LIBRARIES}
)

add_executable(zbserial_listener src/zbserial_listener.cpp)

target_link_libraries(zbserial_listener
  ${catkin_LIBRARIES}
)

add_executable(zbserial_talker src/zbserial_talker.cpp)

target_link_libraries(zbserial_talker
  ${catkin_LIBRARIES}
)