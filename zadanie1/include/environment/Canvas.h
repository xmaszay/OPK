#pragma once

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "types/Geometry.h"

namespace visualization {

class Canvas {
public:
    explicit Canvas(const cv::Mat& map);

    void reset();
    void drawRobot(double x, double y, double resolution);
    void drawHits(const std::vector<geometry::Point2d>& hits, double resolution);
    void show(const std::string& windowName) const;

private:
    cv::Mat originalMap_;
    cv::Mat canvas_;
};

} // namespace visualization