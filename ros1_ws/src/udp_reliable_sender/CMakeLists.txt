cmake_minimum_required(VERSION 3.0.2)
project(udp_reliable_sender)

add_compile_options(-std=c++17 -O2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(udp_reliable_sender_node
  src/udp_reliable_sender_node.cpp
  src/crc32.cpp
  src/chunker.cpp
  src/udp_socket.cpp
  src/compression.cpp
)

target_link_libraries(udp_reliable_sender_node
  ${catkin_LIBRARIES}
)
