cmake_minimum_required(VERSION 3.10)
project(ParticlesFilter)

set(CMAKE_CXX_STANDARD 17)

include_directories(include)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(filter main.cpp src/robot.cpp src/particle.cpp src/particle_filter.cpp)

ament_target_dependencies(filter rclcpp geometry_msgs)

install(TARGETS filter DESTINATION bin)
