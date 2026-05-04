cmake_minimum_required(VERSION 3.8)
project(zadanie1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(robot_lib
  src/Robot.cpp
  src/Lidar.cpp
  src/Environment.cpp
  src/Canvas.cpp
)

target_include_directories(robot_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(robot_lib PUBLIC c_std_99 cxx_std_17)

install(
  TARGETS robot_lib
  EXPORT export_zadanie1
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_zadanie1 HAS_LIBRARY_TARGET)
ament_export_libraries(robot_lib)

ament_package()