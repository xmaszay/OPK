cmake_minimum_required(VERSION 3.8)
project(zadanie1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)

add_library(robot_lib
  src/Robot.cpp
  src/Lidar.cpp
  src/Environment.cpp
  src/Canvas.cpp
)

target_include_directories(robot_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(robot_lib
  ${OpenCV_LIBS}
)

add_executable(zadanie1_main
  src/main.cpp
)

target_include_directories(zadanie1_main PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(zadanie1_main
  robot_lib
  ${OpenCV_LIBS}
)

install(TARGETS
  robot_lib
  zadanie1_main
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include resources
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()