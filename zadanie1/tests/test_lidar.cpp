#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include "Environment.h"
#include "Lidar.h"
#include "geometry/Geometry.h"

void testLidarReturnsSomeHits() {
    environment::Config envConfig;
    envConfig.map_filename = "../maps/map.png";
    envConfig.resolution = 0.05;

    auto env = std::make_shared<environment::Environment>(envConfig);

    lidar::Config lidarConfig;
    lidarConfig.max_range = 5.0;
    lidarConfig.beam_count = 10;
    lidarConfig.first_ray_angle = -1.57;
    lidarConfig.last_ray_angle = 1.57;

    lidar::Lidar lidar(lidarConfig, env);

    geometry::RobotState state;
    state.x = 2.0;
    state.y = 2.0;
    state.theta = 0.0;
    state.velocity = {0.0, 0.0};

    std::vector<geometry::Point2d> hits = lidar.scan(state);

    assert(!hits.empty());
    assert(static_cast<int>(hits.size()) <= lidarConfig.beam_count);
}

int main() {
    testLidarReturnsSomeHits();
    std::cout << "All Lidar tests passed." << std::endl;
    return 0;
}