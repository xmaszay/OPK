#include "Canvas.h"

#include <stdexcept>

namespace visualization {

Canvas::Canvas(const std::string& mapFilename) {
    cv::Mat gray = cv::imread(mapFilename, cv::IMREAD_GRAYSCALE);

    if (gray.empty()) {
        throw std::runtime_error("Map file could not be loaded.");
    }

    cv::cvtColor(gray, originalMap_, cv::COLOR_GRAY2BGR);
    canvas_ = originalMap_.clone();
}

void Canvas::reset() {
    canvas_ = originalMap_.clone();
}

void Canvas::drawRobot(double x, double y, double resolution) {
    int px = static_cast<int>(x / resolution);
    int py = static_cast<int>(y / resolution);

    cv::circle(canvas_, cv::Point(px, py), 5, cv::Scalar(0, 0, 255), -1);
}

void Canvas::drawHits(const std::vector<geometry::Point2d>& hits, double resolution) {
    for (const auto& hit : hits) {
        int px = static_cast<int>(hit.x / resolution);
        int py = static_cast<int>(hit.y / resolution);

        cv::circle(canvas_, cv::Point(px, py), 2, cv::Scalar(0, 255, 0), -1);
    }
}

void Canvas::show(const std::string& windowName) const {
    cv::imshow(windowName, canvas_);
}

} // namespace visualization