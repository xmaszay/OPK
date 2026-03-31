#include <iostream>
#include <memory>

#include <opencv2/opencv.hpp>

#include "environment/Canvas.h"
#include "environment/Environment.h"
#include "environment/Lidar.h"
#include "robot/Robot.h"
#include "types/Geometry.h"

int main() {
    try {
        environment::Config envConfig;
        envConfig.map_filename = "../resources/opk-map.png";
        envConfig.resolution = 0.05;

        auto env = std::make_shared<environment::Environment>(envConfig); // defin env
        auto canvas = std::make_unique<visualization::Canvas>(env->getMap()); // dostane mapu z env
        auto collisionCb = [env](geometry::RobotState state) {
            return env->isOccupied(state.x, state.y);
            };

        lidar::Config lidarConfig;
        lidarConfig.max_range = 13.0; // max dosah
        lidarConfig.beam_count = 360; // pocet lucov
        lidarConfig.first_ray_angle = -CV_PI; // rozsah uhlov 
        lidarConfig.last_ray_angle = CV_PI;

        auto lidar = std::make_shared<lidar::Lidar>(lidarConfig, env); // lidaru dame prisup k prostrediu

        robot::Config robotConfig;
        robotConfig.accelerations = {3.0, 2.0}; // zrychlenie, linear/angular
        robotConfig.emergency_decelerations = {2.0, 2.0}; //spomalovanie
        robotConfig.command_duration = 0.5; // dlzka prikazu
        robotConfig.simulation_period_ms = 20; // ako casto aktualizujeme 

        robot::Robot robot(robotConfig, collisionCb);

        geometry::RobotState startState; // dame robota cca do stredu mapy
        startState.x = env->getWidth() * 0.5;
        startState.y = env->getHeight() * 0.5;
        startState.theta = 0.0;
        startState.velocity = {0.0, 0.0};

        robot.setState(startState);

        std::string windowName = "Robot Simulator"; // okno 
        cv::namedWindow(windowName);

        while (true) {
            geometry::Twist cmd{0.0, 0.0};

            int key = cv::waitKey(30); // caka na vstup z klavesnice 

            if (key == 'w' || key == 'W' || key == 82) {
                cmd.linear = 3.0;
            }
            if (key == 's' || key == 'S' || key == 84) {
                cmd.linear = -3.0;
            }
            if (key == 'a' || key == 'A' || key == 81) {
                cmd.angular = -1.5;
            }
            if (key == 'd' || key == 'D' || key == 83) {
                cmd.angular = 1.5;
            }

            if (key == 27) {
                break;
            }

            robot.setVelocity(cmd); // novy ciel robota

            geometry::RobotState state = robot.getState();  // ziskame aktualnu poziciu a orientaciu robota
            std::vector<geometry::Point2d> hits = lidar->scan(state);  // scan hybeme spolu s robotom 

            canvas->reset(); //vymazanie stareho robota
            canvas->drawRobot(state.x, state.y, state.theta, env->getResolution()); // nakreslenie noveho robota
            canvas->drawHits(hits, env->getResolution()); // nakreslenia aktualnych bodov narazov
            canvas->show(windowName);
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}