#include "environment/Environment.h"

#include <stdexcept>

namespace environment {

Environment::Environment(const Config& config)
    : resolution_(config.resolution) {
    if (resolution_ <= 0.0) {
        throw std::invalid_argument("Resolution must be positive.");
    }

    map_ = cv::imread(config.map_filename, cv::IMREAD_GRAYSCALE); // nacitanie mapy ako grayscale

    if (map_.empty()) {
        throw std::runtime_error("Map file could not be loaded.");
    }
}

bool Environment::isOccupied(double x, double y) const { // ci je bod prekazka alebo nie
    int px = static_cast<int>(x / resolution_);
    int py = static_cast<int>(y / resolution_); // m to px

    if (px < 0 || py < 0 || px >= map_.cols || py >= map_.rows) { // kontrola mimo mapy
        return true;
    }

    return map_.at<unsigned char>/*cita hodnoty px*/(py, px) == 0; // ak je px 0 = prekazka 
}

double Environment::getWidth() const {
    return map_.cols * resolution_; // px to m
}

double Environment::getHeight() const {
    return map_.rows * resolution_;
}

double Environment::getResolution() const {
    return resolution_;
}

const cv::Mat& Environment::getMap() const {
    return map_;
}

} // namespace environment