#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "environment/Environment.h"

class MapNode : public rclcpp::Node
{
public:
    MapNode()
        : Node("map_node")
    {
        this->declare_parameter<std::string>(
            "map_path",
            "/home/peter/Desktop/OPK/opk_ws/src/zadanie1/resources/opk-map.png"
        );
        this->declare_parameter<double>("map_resolution", 0.02);

        environment::Config config;
        config.map_filename = this->get_parameter("map_path").as_string();
        config.resolution = this->get_parameter("map_resolution").as_double();

        env_ = std::make_shared<environment::Environment>(config);

        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "map",
            rclcpp::QoS(1).transient_local().reliable()
        );

        publishMap();

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MapNode::publishMap, this)
        );

        RCLCPP_INFO(
            this->get_logger(),
            "map_node loaded map: %s, resolution: %.3f",
            config.map_filename.c_str(),
            config.resolution
        );
    }

private:
    void publishMap()
    {
        const cv::Mat& image = env_->getMap();

        nav_msgs::msg::OccupancyGrid msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "map";

        msg.info.resolution = env_->getResolution();
        msg.info.width = image.cols;
        msg.info.height = image.rows;

        msg.info.origin.position.x = 0.0;
        msg.info.origin.position.y = 0.0;
        msg.info.origin.position.z = 0.0;
        msg.info.origin.orientation.w = 1.0;

        msg.data.resize(image.cols * image.rows);

        for (int y = 0; y < image.rows; ++y) {
            for (int x = 0; x < image.cols; ++x) {
                unsigned char pixel = image.at<unsigned char>(y, x);
                int index = y * image.cols + x;

                if (pixel == 0) {
                    msg.data[index] = 100;
                } else {
                    msg.data[index] = 0;
                }
            }
        }

        map_pub_->publish(msg);
    }

    std::shared_ptr<environment::Environment> env_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapNode>());
    rclcpp::shutdown();
    return 0;
}