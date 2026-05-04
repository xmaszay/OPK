cmake_minimum_required(VERSION 3.8)
project(cvicenie8_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../zadanie1 ${CMAKE_CURRENT_BINARY_DIR}/zadanie1_build)

add_executable(robot_node src/robot_node.cpp)
target_include_directories(robot_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../zadanie1/include
)
ament_target_dependencies(robot_node rclcpp std_msgs geometry_msgs)
target_link_libraries(robot_node robot_lib)

add_executable(teleop_node src/teleop_node.cpp)
ament_target_dependencies(teleop_node rclcpp geometry_msgs)

install(TARGETS
  robot_node
  teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()