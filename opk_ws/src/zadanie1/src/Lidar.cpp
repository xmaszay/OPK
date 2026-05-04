#include "environment/Lidar.h"

#include <cmath>

namespace lidar {

Lidar::Lidar(const Config& config, std::shared_ptr<environment::Environment> env)
    : config_(config), env_(std::move(env)) {
}

std::vector<geometry::Point2d> Lidar::scan(const geometry::RobotState& state) const { // zoznam bodov kde narazili luce
    std::vector<geometry::Point2d> hits;

    if (config_.beam_count <= 0) { // ak nemame luce
        return hits; 
    }

    double angleStep = 0.0;
    if (config_.beam_count > 1) { // rozdelovanie uhlov rovnomerne medzi lucami
        angleStep = (config_.last_ray_angle - config_.first_ray_angle) /
                    static_cast<double>(config_.beam_count - 1);
    }

    const double step = 0.02;

    for (int i = 0; i < config_.beam_count; ++i) { 
        double relativeAngle = config_.first_ray_angle + i * angleStep;  // urcujeme suradnicove systemy
        double worldAngle = state.theta + relativeAngle;

        for (double distance = 0.0; distance <= config_.max_range; distance = distance + step) { // aproximacia luca
            double x = state.x + distance * std::cos(worldAngle);
            double y = state.y + distance * std::sin(worldAngle);

            if (env_->isOccupied(x, y)) { // kontrola prekazky
                hits.push_back({x, y});
                break;
            }
        }
    }

    return hits;
}

} // namespace lidar