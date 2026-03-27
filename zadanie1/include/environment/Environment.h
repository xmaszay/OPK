#pragma once

#include <string>
#include <opencv2/opencv.hpp>

namespace environment {

struct Config {
    std::string map_filename;
    double resolution;
};

class Environment {
public:
    explicit Environment(const Config& config);

    bool isOccupied(double x, double y) const;
    double getWidth() const;
    double getHeight() const;

    double getResolution() const;
    const cv::Mat& getMap() const;

private:
    cv::Mat map_;
    double resolution_;
};

} // namespace environment