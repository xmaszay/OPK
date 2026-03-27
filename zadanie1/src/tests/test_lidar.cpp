#include <cassert>
#include <iostream>
#include <memory>
#include <vector>

#include "environment/Environment.h"
#include "environment/Lidar.h"
#include "types/Geometry.h"

void testLidarReturnsSomeHits() { // testuje zakladnu funkcionalitu lidaru
    environment::Config envConfig;
    envConfig.map_filename = "../resources/opk-map.png";
    envConfig.resolution = 0.05;

    auto env = std::make_shared<environment::Environment>(envConfig); // nastavim prostredie

    lidar::Config lidarConfig; // nastavim parametre lidaru
    lidarConfig.max_range = 5.0;
    lidarConfig.beam_count = 10;
    lidarConfig.first_ray_angle = -1.57;
    lidarConfig.last_ray_angle = 1.57;

    lidar::Lidar lidar(lidarConfig, env); // vytvorim lidar

    geometry::RobotState state; // zadefinujem si stav robota
    state.x = 100.0;
    state.y = 100.0;
    state.theta = 0.0;
    state.velocity = {0.0, 0.0};

    std::vector<geometry::Point2d> hits = lidar.scan(state);

    assert(!hits.empty()); // ocakavame ze lidar najde nejake miesta narazu lucov
    assert(static_cast<int>(hits.size()) <= lidarConfig.beam_count); // overujem ci pocet zasahov neprekracuje pocet lucov
}

int main() {
    testLidarReturnsSomeHits();
    std::cout << "All Lidar tests passed." << std::endl;
    return 0;
}
