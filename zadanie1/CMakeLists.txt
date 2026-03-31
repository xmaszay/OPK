cmake_minimum_required(VERSION 3.10)
project(robot_simulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(GTest REQUIRED)

add_library(robot_lib
    src/Environment.cpp
    src/Lidar.cpp
    src/Canvas.cpp
    src/Robot.cpp
)

target_include_directories(robot_lib PUBLIC
    include
)

target_link_libraries(robot_lib
    ${OpenCV_LIBS}
)

add_executable(robot_demo
    src/main.cpp
)

target_link_libraries(robot_demo
    robot_lib
)

enable_testing()

# =========================
# Environment test
# =========================
add_executable(test_environment
    src/tests/test_environment.cpp
)

target_link_libraries(test_environment
    robot_lib
    GTest::gtest
    GTest::gtest_main
)

add_test(NAME EnvironmentTest COMMAND test_environment)

# =========================
# Lidar test
# =========================
add_executable(test_lidar
    src/tests/test_lidar.cpp
)

target_link_libraries(test_lidar
    robot_lib
    GTest::gtest
    GTest::gtest_main
)

add_test(NAME LidarTest COMMAND test_lidar)

# =========================
# Robot test
# =========================
add_executable(test_robot
    src/tests/test_robot.cpp
)

target_link_libraries(test_robot
    robot_lib
    # GTest::gtest
    # GTest::gtest_main
)

add_test(NAME RobotTest COMMAND test_robot)