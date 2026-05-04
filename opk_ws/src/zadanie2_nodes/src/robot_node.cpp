#include <memory>
#include <functional>
#include <chrono>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "robot/Robot.h"
#include "environment/Environment.h"
#include "types/Geometry.h"

class RobotNode : public rclcpp::Node
{
public:
    RobotNode()
        : Node("robot_node")
    {
        this->declare_parameter<std::string>(
            "map_path",
            "/home/peter/Desktop/OPK/opk_ws/src/zadanie1/resources/opk-map.png"
        );
        this->declare_parameter<double>("map_resolution", 0.02);

        this->declare_parameter<double>("initial_x", 22.0);
        this->declare_parameter<double>("initial_y", 7.0);
        this->declare_parameter<double>("initial_theta", 0.0);

        this->declare_parameter<bool>("ghost_mode", false);

        this->declare_parameter<double>("linear_acceleration", 3.0);
        this->declare_parameter<double>("angular_acceleration", 2.0);
        this->declare_parameter<double>("linear_emergency_deceleration", 2.0);
        this->declare_parameter<double>("angular_emergency_deceleration", 2.0);
        this->declare_parameter<double>("command_duration", 0.5);
        this->declare_parameter<int>("simulation_period_ms", 20);
        this->declare_parameter<int>("publish_period_ms", 50);

        ghost_mode_ = this->get_parameter("ghost_mode").as_bool();

        environment::Config env_config;
        env_config.map_filename = this->get_parameter("map_path").as_string();
        env_config.resolution = this->get_parameter("map_resolution").as_double();

        env_ = std::make_shared<environment::Environment>(env_config);

       robot::Robot::CollisionCb collision_cb = nullptr;

if (!ghost_mode_) {
    collision_cb = [this](geometry::RobotState state) {
        return env_->isOccupied(state.x, state.y);
    };
} else {
    collision_cb = [](geometry::RobotState) {
        return false;
    };
}

        robot::Config robot_config;
        robot_config.accelerations.linear =
            this->get_parameter("linear_acceleration").as_double();
        robot_config.accelerations.angular =
            this->get_parameter("angular_acceleration").as_double();

        robot_config.emergency_decelerations.linear =
            this->get_parameter("linear_emergency_deceleration").as_double();
        robot_config.emergency_decelerations.angular =
            this->get_parameter("angular_emergency_deceleration").as_double();

        robot_config.command_duration =
            this->get_parameter("command_duration").as_double();

        robot_config.simulation_period_ms =
            this->get_parameter("simulation_period_ms").as_int();

        robot_ = std::make_unique<robot::Robot>(robot_config, collision_cb, true);

        geometry::RobotState initial_state;
        initial_state.x = this->get_parameter("initial_x").as_double();
        initial_state.y = this->get_parameter("initial_y").as_double();
        initial_state.theta = this->get_parameter("initial_theta").as_double();
        initial_state.velocity = {0.0, 0.0};

        if (!ghost_mode_ && env_->isOccupied(initial_state.x, initial_state.y)) {
            RCLCPP_WARN(
                this->get_logger(),
                "Initial position %.2f %.2f is occupied. Searching nearest free position...",
                initial_state.x,
                initial_state.y
            );

            initial_state = findNearestFreeState(initial_state);

            RCLCPP_WARN(
                this->get_logger(),
                "Robot moved to nearest free position %.2f %.2f",
                initial_state.x,
                initial_state.y
            );
        }

        robot_->setState(initial_state);

        std::string ns = this->get_namespace();

        if (!ns.empty() && ns[0] == '/') {
            ns.erase(0, 1);
        }

        if (ns.empty()) {
            base_frame_id_ = "base_link";
        } else {
            base_frame_id_ = ns + "/base_link";
        }

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel",
            10,
            std::bind(&RobotNode::cmdVelCallback, this, std::placeholders::_1)
        );

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "odom",
            10
        );

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "robot_marker",
            10
        );

        int publish_period_ms = this->get_parameter("publish_period_ms").as_int();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(publish_period_ms),
            std::bind(&RobotNode::publishState, this)
        );

        RCLCPP_INFO(
            this->get_logger(),
            "robot_node started. Namespace: %s, base_frame: %s, ghost_mode: %s",
            this->get_namespace(),
            base_frame_id_.c_str(),
            ghost_mode_ ? "true" : "false"
        );
    }

private:
    geometry::RobotState findNearestFreeState(const geometry::RobotState& original_state)
    {
        geometry::RobotState free_state = original_state;

        const double step = env_->getResolution();
        const double max_radius = 5.0;

        for (double radius = step; radius <= max_radius; radius += step) {
            for (double dx = -radius; dx <= radius; dx += step) {
                for (double dy = -radius; dy <= radius; dy += step) {
                    double x = original_state.x + dx;
                    double y = original_state.y + dy;

                    if (!env_->isOccupied(x, y)) {
                        free_state.x = x;
                        free_state.y = y;
                        free_state.velocity = {0.0, 0.0};
                        return free_state;
                    }
                }
            }
        }

        RCLCPP_ERROR(
            this->get_logger(),
            "Could not find free spawn position. Keeping original position."
        );

        return original_state;
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        geometry::Twist velocity;
        velocity.linear = msg->linear.x;
        velocity.angular = msg->angular.z;

        robot_->setVelocity(velocity);
    }

    void publishState()
    {
        geometry::RobotState state = robot_->getState();

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, state.theta);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = this->get_clock()->now();
        odom.header.frame_id = "map";
        odom.child_frame_id = base_frame_id_;

        odom.pose.pose.position.x = state.x;
        odom.pose.pose.position.y = state.y;
        odom.pose.pose.position.z = 0.0;

        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        odom.twist.twist.linear.x = state.velocity.linear;
        odom.twist.twist.angular.z = state.velocity.angular;

        odom_pub_->publish(odom);

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = base_frame_id_;

        transform.transform.translation.x = state.x;
        transform.transform.translation.y = state.y;
        transform.transform.translation.z = 0.0;

        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transform);

        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = "map";

        marker.ns = this->get_namespace();
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = state.x;
        marker.pose.position.y = state.y;
        marker.pose.position.z = 0.10;

        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        marker.scale.x = 0.7;
        marker.scale.y = 0.7;
        marker.scale.z = 0.2;

        if (ghost_mode_) {
            marker.color.r = 0.7f;
            marker.color.g = 0.2f;
            marker.color.b = 1.0f;
            marker.color.a = 0.8f;
        } else {
            marker.color.r = 0.0f;
            marker.color.g = 0.3f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0f;
        }

        if (!ghost_mode_ && robot_->isInCollision()) {
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
        }

        marker_pub_->publish(marker);
    }

    std::shared_ptr<environment::Environment> env_;
    std::unique_ptr<robot::Robot> robot_;

    bool ghost_mode_ = false;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string base_frame_id_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotNode>());
    rclcpp::shutdown();
    return 0;
}