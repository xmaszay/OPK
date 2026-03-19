#include <iostream>
#include <memory>
#include <vector>

#include <opencv2/opencv.hpp>

#include "Canvas.h"
#include "Environment.h"
#include "Lidar.h"
#include "geometry/Geometry.h"

// globalne veci
std::vector<geometry::RobotState> robotStates;

std::shared_ptr<environment::Environment> g_env;
std::shared_ptr<lidar::Lidar> g_lidar;
std::unique_ptr<visualization::Canvas> g_canvas;

double g_resolution;
std::string g_windowName = "Robot Simulator";

// mouse callback
void onMouse(int event, int x, int y, int, void*) {
    if (event != cv::EVENT_LBUTTONDOWN)
        return;

    double worldX = x * g_resolution;
    double worldY = y * g_resolution;

    geometry::RobotState state;
    state.x = worldX;
    state.y = worldY;
    state.theta = 0.0;
    state.velocity = {0.0, 0.0};

    robotStates.push_back(state);

    g_canvas->reset();

    for (const auto& s : robotStates) {
        auto hits = g_lidar->scan(s);
        g_canvas->drawRobot(s.x, s.y, g_resolution);
        g_canvas->drawHits(hits, g_resolution);
    }

    g_canvas->show(g_windowName);
}

int main() {
    try {
        const std::string mapFilename = "../maps/map.png";
        g_resolution = 0.05;

        environment::Config envConfig;
        envConfig.map_filename = mapFilename;
        envConfig.resolution = g_resolution;

        g_env = std::make_shared<environment::Environment>(envConfig);

        lidar::Config lidarConfig;
        lidarConfig.max_range = 13;
        lidarConfig.beam_count = 360;
        lidarConfig.first_ray_angle = -M_PI;
        lidarConfig.last_ray_angle = M_PI;

        g_lidar = std::make_shared<lidar::Lidar>(lidarConfig, g_env);

        g_canvas = std::make_unique<visualization::Canvas>(mapFilename);

        cv::namedWindow(g_windowName);
        cv::setMouseCallback(g_windowName, onMouse);

        g_canvas->show(g_windowName);

        while (true) {
            int key = cv::waitKey(10);
            if (key == 27) // ESC
                break;
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}