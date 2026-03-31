#include <cassert>
#include <cmath>
#include <iostream>
#include <thread>

#include "robot/Robot.h"

class TestableRobot : public robot::Robot {
public:
    TestableRobot(const robot::Config& config)
        : robot::Robot(config, nullptr, false) {
    }

    using robot::Robot::update;
};

void testUpdateMovesRobotForward() {
    robot::Config config;
    config.accelerations = {1.0, 1.0};
    config.emergency_decelerations = {2.0, 2.0};
    config.command_duration = 1.0;
    config.simulation_period_ms = 100;

    TestableRobot robot(config);

    geometry::Twist command;
    command.linear = 1.0;
    command.angular = 0.0;

    robot.update(command, 1.0);

    geometry::RobotState state = robot.getState();

    assert(state.x > 0.0);
    assert(std::abs(state.y) < 1e-9);
}

void testUpdateChangesOrientation() {
    robot::Config config;
    config.accelerations = {1.0, 1.0};
    config.emergency_decelerations = {2.0, 2.0};
    config.command_duration = 1.0;
    config.simulation_period_ms = 100;

    TestableRobot robot(config);

    geometry::Twist command;
    command.linear = 0.0;
    command.angular = 1.0;

    robot.update(command, 1.0);

    geometry::RobotState state = robot.getState();

    assert(state.theta > 0.0);
}

void testRobotStopsWithoutNewCommand() {
    robot::Config config;
    config.accelerations = {1.0, 1.0};
    config.emergency_decelerations = {5.0, 5.0};
    config.command_duration = 0.2;
    config.simulation_period_ms = 50;

    robot::Robot robot(config);

    geometry::Twist command;
    command.linear = 1.0;
    command.angular = 0.0;

    robot.setVelocity(command);

    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    double moving_velocity = robot.getState().velocity.linear;

    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    double stopped_velocity = robot.getState().velocity.linear;

    assert(moving_velocity > 0.0);
    assert(stopped_velocity < moving_velocity);
}

int main() {
    testUpdateMovesRobotForward();
    testUpdateChangesOrientation();
    testRobotStopsWithoutNewCommand();

    std::cout << "All Robot tests passed." << std::endl;
    return 0;
}