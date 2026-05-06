#include <memory>
#include <functional>
#include <chrono>
#include <string>
#include <cmath>
#include <vector>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

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
        declareRequiredParameters();

        ghost_mode_ = getRequiredParameter<bool>("ghost_mode");
        robot_radius_ = getRequiredParameter<double>("robot_radius");

        environment::Config env_config;
        env_config.map_filename = getRequiredParameter<std::string>("map_path");
        env_config.resolution = getRequiredParameter<double>("map_resolution");

        env_ = std::make_shared<environment::Environment>(env_config);

        loadObstaclesFromParameters();

        robot::Robot::CollisionCb collision_cb = nullptr;

        if (!ghost_mode_) {
            collision_cb = [this](geometry::RobotState state) {
                return isOccupiedByMapOrObstacle(state.x, state.y);
            };
        } else {
            collision_cb = [](geometry::RobotState) {
                return false;
            };
        }

        robot::Config robot_config;
        robot_config.accelerations.linear =
            getRequiredParameter<double>("linear_acceleration");
        robot_config.accelerations.angular =
            getRequiredParameter<double>("angular_acceleration");

        robot_config.emergency_decelerations.linear =
            getRequiredParameter<double>("linear_emergency_deceleration");
        robot_config.emergency_decelerations.angular =
            getRequiredParameter<double>("angular_emergency_deceleration");

        robot_config.command_duration =
            getRequiredParameter<double>("command_duration");

        robot_config.simulation_period_ms =
            getRequiredParameter<int>("simulation_period_ms");

        robot_ = std::make_unique<robot::Robot>(robot_config, collision_cb, true);

        geometry::RobotState initial_state;
        initial_state.x = getRequiredParameter<double>("initial_x");
        initial_state.y = getRequiredParameter<double>("initial_y");
        initial_state.theta = getRequiredParameter<double>("initial_theta");
        initial_state.velocity = {0.0, 0.0};

        if (!ghost_mode_ && isOccupiedByMapOrObstacle(initial_state.x, initial_state.y)) {
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

        int publish_period_ms = getRequiredParameter<int>("publish_period_ms");

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(publish_period_ms),
            std::bind(&RobotNode::publishState, this)
        );

        RCLCPP_INFO(
            this->get_logger(),
            "robot_node started. Namespace: %s, base_frame: %s, ghost_mode: %s, robot_radius: %.2f",
            this->get_namespace(),
            base_frame_id_.c_str(),
            ghost_mode_ ? "true" : "false",
            robot_radius_
        );
    }

private:
    struct CircleObstacle
    {
        double x;
        double y;
        double radius;
    };

    struct RectangleObstacle
    {
        double x;
        double y;
        double width;
        double height;
    };

    void declareRequiredParameters()
    {
        this->declare_parameter<std::string>("map_path");
        this->declare_parameter<double>("map_resolution");

        this->declare_parameter<double>("initial_x");
        this->declare_parameter<double>("initial_y");
        this->declare_parameter<double>("initial_theta");

        this->declare_parameter<bool>("ghost_mode");
        this->declare_parameter<double>("robot_radius");

        this->declare_parameter<double>("linear_acceleration");
        this->declare_parameter<double>("angular_acceleration");
        this->declare_parameter<double>("linear_emergency_deceleration");
        this->declare_parameter<double>("angular_emergency_deceleration");
        this->declare_parameter<double>("command_duration");
        this->declare_parameter<int>("simulation_period_ms");
        this->declare_parameter<int>("publish_period_ms");

        this->declare_parameter<std::vector<double>>("circle_obstacles_x");
        this->declare_parameter<std::vector<double>>("circle_obstacles_y");
        this->declare_parameter<std::vector<double>>("circle_obstacles_radius");

        this->declare_parameter<std::vector<double>>("rectangle_obstacles_x");
        this->declare_parameter<std::vector<double>>("rectangle_obstacles_y");
        this->declare_parameter<std::vector<double>>("rectangle_obstacles_width");
        this->declare_parameter<std::vector<double>>("rectangle_obstacles_height");
    }

    template<typename T>
    T getRequiredParameter(const std::string& name)
    {
        T value;

        if (!this->get_parameter(name, value)) {
            throw std::runtime_error("Missing required ROS parameter: " + name);
        }

        return value;
    }

    void loadObstaclesFromParameters()
    {
        auto circle_x = getRequiredParameter<std::vector<double>>("circle_obstacles_x");
        auto circle_y = getRequiredParameter<std::vector<double>>("circle_obstacles_y");
        auto circle_r = getRequiredParameter<std::vector<double>>("circle_obstacles_radius");

        if (circle_x.size() != circle_y.size() || circle_x.size() != circle_r.size()) {
            throw std::runtime_error("Circle obstacle parameter arrays have different sizes.");
        }

        for (size_t i = 0; i < circle_x.size(); ++i) {
            if (circle_r[i] <= 0.0) {
                throw std::runtime_error("Circle obstacle radius must be positive.");
            }

            circle_obstacles_.push_back({
                circle_x[i],
                circle_y[i],
                circle_r[i]
            });
        }

        auto rect_x = getRequiredParameter<std::vector<double>>("rectangle_obstacles_x");
        auto rect_y = getRequiredParameter<std::vector<double>>("rectangle_obstacles_y");
        auto rect_w = getRequiredParameter<std::vector<double>>("rectangle_obstacles_width");
        auto rect_h = getRequiredParameter<std::vector<double>>("rectangle_obstacles_height");

        if (rect_x.size() != rect_y.size() ||
            rect_x.size() != rect_w.size() ||
            rect_x.size() != rect_h.size()) {
            throw std::runtime_error("Rectangle obstacle parameter arrays have different sizes.");
        }

        for (size_t i = 0; i < rect_x.size(); ++i) {
            if (rect_w[i] <= 0.0 || rect_h[i] <= 0.0) {
                throw std::runtime_error("Rectangle obstacle dimensions must be positive.");
            }

            rectangle_obstacles_.push_back({
                rect_x[i],
                rect_y[i],
                rect_w[i],
                rect_h[i]
            });
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Loaded %zu circle obstacles and %zu rectangle obstacles.",
            circle_obstacles_.size(),
            rectangle_obstacles_.size()
        );
    }

    bool isInsideCircleObstacle(double x, double y) const
    {
        for (const auto& obstacle : circle_obstacles_) {
            double dx = x - obstacle.x;
            double dy = y - obstacle.y;

            if (std::sqrt(dx * dx + dy * dy) <= obstacle.radius + robot_radius_) {
                return true;
            }
        }

        return false;
    }

    bool isInsideRectangleObstacle(double x, double y) const
    {
        for (const auto& obstacle : rectangle_obstacles_) {
            bool inside =
                x >= obstacle.x - obstacle.width * 0.5 - robot_radius_ &&
                x <= obstacle.x + obstacle.width * 0.5 + robot_radius_ &&
                y >= obstacle.y - obstacle.height * 0.5 - robot_radius_ &&
                y <= obstacle.y + obstacle.height * 0.5 + robot_radius_;

            if (inside) {
                return true;
            }
        }

        return false;
    }

    bool isOccupiedByMapOrObstacle(double x, double y) const
    {
        if (env_->isOccupied(x, y)) {
            return true;
        }

        if (isInsideCircleObstacle(x, y)) {
            return true;
        }

        if (isInsideRectangleObstacle(x, y)) {
            return true;
        }

        return false;
    }

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

                    if (!isOccupiedByMapOrObstacle(x, y)) {
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
    }

    std::shared_ptr<environment::Environment> env_;
    std::unique_ptr<robot::Robot> robot_;

    bool ghost_mode_ = false;
    double robot_radius_ = 0.0;

    std::vector<CircleObstacle> circle_obstacles_;
    std::vector<RectangleObstacle> rectangle_obstacles_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string base_frame_id_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotNode>());
    rclcpp::shutdown();
    return 0;
}