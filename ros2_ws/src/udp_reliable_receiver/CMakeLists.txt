cmake_minimum_required(VERSION 3.8)
project(udp_reliable_receiver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(udp_reliable_receiver_node
  src/udp_reliable_receiver_node.cpp
  src/crc32.cpp
  src/udp_socket.cpp
  src/frame_buffer.cpp
  src/stats.cpp
  src/compression.cpp
)

ament_target_dependencies(udp_reliable_receiver_node
  rclcpp
  sensor_msgs
)

install(TARGETS
  udp_reliable_receiver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
