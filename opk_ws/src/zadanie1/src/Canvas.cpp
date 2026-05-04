#include "environment/Canvas.h"

#include <stdexcept>

namespace visualization {

Canvas::Canvas(const cv::Mat& map) {
    if (map.empty()) {
        throw std::runtime_error("Map image is empty.");
    }

    cv::cvtColor(map, originalMap_, cv::COLOR_GRAY2BGR);
    canvas_ = originalMap_.clone();
}

void Canvas::reset() {
    canvas_ = originalMap_.clone();
}

void Canvas::drawRobot(double x, double y, double theta, double resolution) {
    int px = static_cast<int>(x / resolution);
    int py = static_cast<int>(y / resolution);

    // telo robota
    cv::circle(canvas_, cv::Point(px, py), 8, cv::Scalar(0, 0, 255), -1);

    // smer robota (sipka)
    int arrowLength = 20;

    int endX = static_cast<int>(px + arrowLength * std::cos(theta));
    int endY = static_cast<int>(py + arrowLength * std::sin(theta));

    cv::arrowedLine(
        canvas_,
        cv::Point(px, py),
        cv::Point(endX, endY),
        cv::Scalar(255, 0, 0), // modra sipka
        2
    );
}

void Canvas::drawHits(const std::vector<geometry::Point2d>& hits, double resolution) {
    for (const auto& hit : hits) { //prejde vsetky body 
        int px = static_cast<int>(hit.x / resolution);
        int py = static_cast<int>(hit.y / resolution);

        cv::circle(canvas_, cv::Point(px, py), 2, cv::Scalar(0, 255, 0), -1);// vykresli body
    }
}

void Canvas::show(const std::string& windowName) const {
    cv::imshow(windowName, canvas_);
}

} // namespace visualization