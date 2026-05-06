#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <limits>
#include <chrono>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "environment/Environment.h"

class LidarNode : public rclcpp::Node
{
public:
    LidarNode()
        : Node("lidar_node")
    {
        this->declare_parameter<std::string>("map_path");
        this->declare_parameter<double>("map_resolution");
        this->declare_parameter<std::string>("base_frame_id");

        this->declare_parameter<double>("max_range");
        this->declare_parameter<int>("beam_count");
        this->declare_parameter<double>("first_ray_angle");
        this->declare_parameter<double>("last_ray_angle");
        this->declare_parameter<int>("publish_period_ms");

        environment::Config env_config;
        env_config.map_filename = getRequiredParameter<std::string>("map_path");
        env_config.resolution = getRequiredParameter<double>("map_resolution");

        env_ = std::make_shared<environment::Environment>(env_config);

        base_frame_id_ = getRequiredParameter<std::string>("base_frame_id");
        max_range_ = getRequiredParameter<double>("max_range");
        beam_count_ = getRequiredParameter<int>("beam_count");
        first_ray_angle_ = getRequiredParameter<double>("first_ray_angle");
        last_ray_angle_ = getRequiredParameter<double>("last_ray_angle");

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom",
            10,
            std::bind(&LidarNode::odomCallback, this, std::placeholders::_1)
        );

        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "scan",
            10
        );

        int publish_period_ms = getRequiredParameter<int>("publish_period_ms");

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(publish_period_ms),
            std::bind(&LidarNode::publishScan, this)
        );

        RCLCPP_INFO(
            this->get_logger(),
            "lidar_node started. base_frame: %s, max_range: %.2f, beam_count: %d",
            base_frame_id_.c_str(),
            max_range_,
            beam_count_
        );
    }

private:
    template<typename T>
    T getRequiredParameter(const std::string& name)
    {
        T value;
        if (!this->get_parameter(name, value)) {
            throw std::runtime_error("Missing required ROS parameter: " + name);
        }
        return value;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;

        const auto& q = msg->pose.pose.orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        robot_theta_ = std::atan2(siny_cosp, cosy_cosp);

        has_odom_ = true;
    }

    double traceSingleRay(double world_angle) const
    {
        const double step = 0.02;

        for (double distance = 0.0; distance <= max_range_; distance += step) {
            double x = robot_x_ + distance * std::cos(world_angle);
            double y = robot_y_ + distance * std::sin(world_angle);

            if (env_->isOccupied(x, y)) {
                return distance;
            }
        }

        return std::numeric_limits<float>::infinity();
    }

    void publishScan()
    {
        if (!has_odom_) {
            return;
        }

        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = this->get_clock()->now();
        scan.header.frame_id = base_frame_id_;

        scan.angle_min = first_ray_angle_;
        scan.angle_max = last_ray_angle_;
        scan.range_min = 0.0;
        scan.range_max = max_range_;
        scan.time_increment = 0.0;
        scan.scan_time = 0.1;

        if (beam_count_ <= 1) {
            scan.angle_increment = 0.0;
            scan.ranges.resize(1);

            double world_angle = robot_theta_ + first_ray_angle_;
            scan.ranges[0] = static_cast<float>(traceSingleRay(world_angle));
        } else {
            scan.angle_increment =
                (last_ray_angle_ - first_ray_angle_) / static_cast<double>(beam_count_ - 1);

            scan.ranges.resize(beam_count_);

            for (int i = 0; i < beam_count_; ++i) {
                double relative_angle = first_ray_angle_ + i * scan.angle_increment;
                double world_angle = robot_theta_ + relative_angle;

                scan.ranges[i] = static_cast<float>(traceSingleRay(world_angle));
            }
        }

        scan_pub_->publish(scan);
    }

    std::shared_ptr<environment::Environment> env_;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_theta_ = 0.0;
    bool has_odom_ = false;

    std::string base_frame_id_;
    double max_range_ = 0.0;
    int beam_count_ = 0;
    double first_ray_angle_ = 0.0;
    double last_ray_angle_ = 0.0;

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