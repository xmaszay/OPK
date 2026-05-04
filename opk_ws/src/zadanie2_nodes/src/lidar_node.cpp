#include <memory>
#include <vector>
#include <cmath>
#include <limits>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "environment/Environment.h"
#include "environment/Lidar.h"
#include "types/Geometry.h"

class LidarNode : public rclcpp::Node
{
public:
    LidarNode()
        : Node("lidar_node")
    {
        this->declare_parameter<std::string>(
            "map_path",
            "/home/peter/Desktop/OPK/opk_ws/src/zadanie1/resources/opk-map.png"
        );
        this->declare_parameter<double>("map_resolution", 0.02);

        this->declare_parameter<std::string>("base_frame_id", "base_link");

        this->declare_parameter<double>("max_range", 8.0);
        this->declare_parameter<int>("beam_count", 360);
        this->declare_parameter<double>("first_ray_angle", -M_PI);
        this->declare_parameter<double>("last_ray_angle", M_PI);
        this->declare_parameter<int>("publish_period_ms", 100);

        environment::Config env_config;
        env_config.map_filename = this->get_parameter("map_path").as_string();
        env_config.resolution = this->get_parameter("map_resolution").as_double();

        env_ = std::make_shared<environment::Environment>(env_config);

        lidar::Config lidar_config;
        lidar_config.max_range = this->get_parameter("max_range").as_double();
        lidar_config.beam_count = this->get_parameter("beam_count").as_int();
        lidar_config.first_ray_angle = this->get_parameter("first_ray_angle").as_double();
        lidar_config.last_ray_angle = this->get_parameter("last_ray_angle").as_double();

        lidar_ = std::make_unique<lidar::Lidar>(lidar_config, env_);

        base_frame_id_ = this->get_parameter("base_frame_id").as_string();

        max_range_ = lidar_config.max_range;
        beam_count_ = lidar_config.beam_count;
        first_ray_angle_ = lidar_config.first_ray_angle;
        last_ray_angle_ = lidar_config.last_ray_angle;

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom",
            10,
            std::bind(&LidarNode::odomCallback, this, std::placeholders::_1)
        );

        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "scan",
            10
        );

        int publish_period_ms = this->get_parameter("publish_period_ms").as_int();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(publish_period_ms),
            std::bind(&LidarNode::publishScan, this)
        );

        RCLCPP_INFO(
            this->get_logger(),
            "lidar_node started. base_frame: %s",
            base_frame_id_.c_str()
        );
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        last_state_.x = msg->pose.pose.position.x;
        last_state_.y = msg->pose.pose.position.y;

        const auto& q = msg->pose.pose.orientation;

        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        last_state_.theta = std::atan2(siny_cosp, cosy_cosp);

        last_state_.velocity.linear = msg->twist.twist.linear.x;
        last_state_.velocity.angular = msg->twist.twist.angular.z;

        has_odom_ = true;
    }

    void publishScan()
    {
        if (!has_odom_) {
            return;
        }

        std::vector<geometry::Point2d> hits = lidar_->scan(last_state_);

        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = this->get_clock()->now();
        scan.header.frame_id = base_frame_id_;

        scan.angle_min = first_ray_angle_;
        scan.angle_max = last_ray_angle_;

        if (beam_count_ > 1) {
            scan.angle_increment =
                (last_ray_angle_ - first_ray_angle_) / static_cast<double>(beam_count_ - 1);
        } else {
            scan.angle_increment = 0.0;
        }

        scan.time_increment = 0.0;
        scan.scan_time = 0.1;
        scan.range_min = 0.0;
        scan.range_max = max_range_;

        scan.ranges.resize(beam_count_, std::numeric_limits<float>::infinity());

        for (int i = 0; i < beam_count_ && i < static_cast<int>(hits.size()); ++i) {
            double dx = hits[i].x - last_state_.x;
            double dy = hits[i].y - last_state_.y;
            double distance = std::sqrt(dx * dx + dy * dy);

            scan.ranges[i] = static_cast<float>(distance);
        }

        scan_pub_->publish(scan);
    }

    std::shared_ptr<environment::Environment> env_;
    std::unique_ptr<lidar::Lidar> lidar_;

    geometry::RobotState last_state_;
    bool has_odom_ = false;

    std::string base_frame_id_;

    double max_range_;
    int beam_count_;
    double first_ray_angle_;
    double last_ray_angle_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarNode>());
    rclcpp::shutdown();
    return 0;
}