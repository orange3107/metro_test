cmake_minimum_required(VERSION 3.10)
project(VehicleSimulation)

set(CMAKE_CXX_STANDARD 17)

include_directories(include)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(vehicle main.cpp src/robot.cpp src/particle.cpp src/particle_filter.cpp)

ament_target_dependencies(vehicle rclcpp geometry_msgs)

install(TARGETS vehicle DESTINATION bin)
