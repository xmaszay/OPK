cmake_minimum_required(VERSION 3.8)
project(zadanie2_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(zadanie2_interfaces REQUIRED)

set(ZADANIE1_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../zadanie1)

add_library(zadanie1_core
  ${ZADANIE1_DIR}/src/Robot.cpp
  ${ZADANIE1_DIR}/src/Lidar.cpp
  ${ZADANIE1_DIR}/src/Environment.cpp
  ${ZADANIE1_DIR}/src/Canvas.cpp
)

target_include_directories(zadanie1_core PUBLIC
  ${ZADANIE1_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(zadanie1_core
  ${OpenCV_LIBS}
)

target_compile_features(zadanie1_core PUBLIC cxx_std_17)


add_executable(robot_node
  src/robot_node.cpp
)

target_include_directories(robot_node PUBLIC
  include
  ${ZADANIE1_DIR}/include
)

ament_target_dependencies(robot_node
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
)

target_link_libraries(robot_node
  zadanie1_core
  ${OpenCV_LIBS}
)


add_executable(map_node
  src/map_node.cpp
)

target_include_directories(map_node PUBLIC
  include
  ${ZADANIE1_DIR}/include
)

ament_target_dependencies(map_node
  rclcpp
  nav_msgs
)

target_link_libraries(map_node
  zadanie1_core
  ${OpenCV_LIBS}
)


add_executable(lidar_node
  src/lidar_node.cpp
)

target_include_directories(lidar_node PUBLIC
  include
  ${ZADANIE1_DIR}/include
)

ament_target_dependencies(lidar_node
  rclcpp
  nav_msgs
  sensor_msgs
)

target_link_libraries(lidar_node
  zadanie1_core
  ${OpenCV_LIBS}
)


add_executable(game_node
  src/game_node.cpp
  src/GameLogic.cpp
)

target_include_directories(game_node PUBLIC
  include
  ${ZADANIE1_DIR}/include
)

ament_target_dependencies(game_node
  rclcpp
  nav_msgs
  zadanie2_interfaces
)

target_link_libraries(game_node
  zadanie1_core
  ${OpenCV_LIBS}
)


add_executable(keyboard_teleop_node
  src/keyboard_teleop_node.cpp
)

ament_target_dependencies(keyboard_teleop_node
  rclcpp
  geometry_msgs
)


add_executable(bot_node
  src/bot_node.cpp
)

ament_target_dependencies(bot_node
  rclcpp
  geometry_msgs
  nav_msgs
  zadanie2_interfaces
)


add_executable(visualization_node
  src/visualization_node.cpp
)

ament_target_dependencies(visualization_node
  rclcpp
  nav_msgs
  sensor_msgs
  visualization_msgs
  zadanie2_interfaces
)

add_executable(web_node
  src/web_node.cpp
)

ament_target_dependencies(web_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  zadanie2_interfaces
)

install(TARGETS
  robot_node
  map_node
  lidar_node
  game_node
  keyboard_teleop_node
  bot_node
  visualization_node
  web_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()