#include <iostream>
#include <memory>
#include <vector>

#include <opencv2/opencv.hpp>

#include "environment/Canvas.h"
#include "environment/Environment.h"
#include "environment/Lidar.h"
#include "types/Geometry.h"

struct AppState {
    std::vector<geometry::RobotState> robotStates; // ukladam vsetky kliknute pozicie
    std::shared_ptr<environment::Environment> env;
    std::shared_ptr<lidar::Lidar> lidar;
    std::unique_ptr<visualization::Canvas> canvas; // canvas ma 1 vlastnika tak staci unique
    std::string windowName;
};

void onMouse(int event, int x, int y, int, void* userdata) {
    if (event != cv::EVENT_LBUTTONDOWN) { // kontrola laveho kliku
        return;
    }

    auto* app = static_cast<AppState*>(userdata);
    if (app == nullptr || app->env == nullptr || app->lidar == nullptr || app->canvas == nullptr) {
        return;
    }

    double resolution = app->env->getResolution();

    double worldX = x * resolution;
    double worldY = y * resolution; // px to mm

    geometry::RobotState state; // vytvaranie stavov robota
    state.x = worldX;
    state.y = worldY;
    state.theta = 0.0;
    state.velocity = {0.0, 0.0};

    app->robotStates.push_back(state); 

    app->canvas->reset();

    for (const auto& robotState : app->robotStates) {
        std::vector<geometry::Point2d> hits = app->lidar->scan(robotState);
        app->canvas->drawRobot(robotState.x, robotState.y, resolution);
        app->canvas->drawHits(hits, resolution);
    }

    app->canvas->show(app->windowName);
}

int main() {
    try {
        environment::Config envConfig;
        envConfig.map_filename = "../resources/opk-map.png";
        envConfig.resolution = 0.05;

        auto env = std::make_shared<environment::Environment>(envConfig);

        lidar::Config lidarConfig; // konfiguracia lidar
        lidarConfig.max_range = 13.0; // maximalny dosah 
        lidarConfig.beam_count = 360; // pocet lucov 
        lidarConfig.first_ray_angle = -CV_PI; // prvy a posledny luc je v rozsahu 360 stupnov
        lidarConfig.last_ray_angle = CV_PI;

        auto lidar = std::make_shared<lidar::Lidar>(lidarConfig, env);
        auto canvas = std::make_unique<visualization::Canvas>(env->getMap());

        AppState app;
        app.env = env;
        app.lidar = lidar;
        app.canvas = std::move(canvas);
        app.windowName = "Robot Simulator";

        cv::namedWindow(app.windowName);
        cv::setMouseCallback(app.windowName, onMouse, &app);

        app.canvas->show(app.windowName);

        while (true) {
            int key = cv::waitKey(10);
            if (key == 27) {
                break;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl; // ak nastane chyba vypise sa error
        return 1;
    }

    return 0;
}