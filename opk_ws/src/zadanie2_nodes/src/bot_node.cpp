#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "zadanie2_interfaces/msg/game_state.hpp"

struct Target
{
    int id;
    double x;
    double y;
};

class BotNode : public rclcpp::Node
{
public:
    BotNode()
        : Node("bot_node")
    {
        this->declare_parameter<int>("max_capacity");
        this->declare_parameter<double>("station_x");
        this->declare_parameter<double>("station_y");
        this->declare_parameter<double>("target_distance");
        this->declare_parameter<double>("linear_speed");
        this->declare_parameter<double>("angular_gain");
        this->declare_parameter<double>("angle_tolerance");

        max_capacity_ = getRequiredParameter<int>("max_capacity");
        station_x_ = getRequiredParameter<double>("station_x");
        station_y_ = getRequiredParameter<double>("station_y");
        target_distance_ = getRequiredParameter<double>("target_distance");
        linear_speed_ = getRequiredParameter<double>("linear_speed");
        angular_gain_ = getRequiredParameter<double>("angular_gain");
        angle_tolerance_ = getRequiredParameter<double>("angle_tolerance");

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/player2/odom",
            10,
            std::bind(&BotNode::odomCallback, this, std::placeholders::_1)
        );

        game_state_sub_ = this->create_subscription<zadanie2_interfaces::msg::GameState>(
            "/game_state",
            10,
            std::bind(&BotNode::gameStateCallback, this, std::placeholders::_1)
        );

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/player2/cmd_vel",
            10
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&BotNode::controlLoop, this)
        );

        RCLCPP_INFO(this->get_logger(), "bot_node started for player2");
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
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;

        const auto& q = msg->pose.pose.orientation;

        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        theta_ = std::atan2(siny_cosp, cosy_cosp);

        has_odom_ = true;
    }

    void gameStateCallback(const zadanie2_interfaces::msg::GameState::SharedPtr msg)
    {
        targets_.clear();

        for (size_t i = 0; i < msg->trash_id.size(); ++i) {
            if (i >= msg->trash_x.size() ||
                i >= msg->trash_y.size() ||
                i >= msg->trash_collected.size()) {
                continue;
            }

            if (msg->trash_collected[i]) {
                continue;
            }

            Target target;
            target.id = msg->trash_id[i];
            target.x = msg->trash_x[i];
            target.y = msg->trash_y[i];

            targets_[target.id] = target;
        }

        station_x_ = msg->station_x;
        station_y_ = msg->station_y;
    }

    double normalizeAngle(double angle) const
    {
        while (angle > M_PI) {
            angle -= 2.0 * M_PI;
        }

        while (angle < -M_PI) {
            angle += 2.0 * M_PI;
        }

        return angle;
    }

    double distance(double x1, double y1, double x2, double y2) const
    {
        double dx = x1 - x2;
        double dy = y1 - y2;
        return std::sqrt(dx * dx + dy * dy);
    }

    bool findNearestTrash(Target& nearest)
    {
        if (targets_.empty()) {
            return false;
        }

        bool found = false;
        double best_distance = 1e9;

        for (const auto& pair : targets_) {
            const auto& target = pair.second;

            double d = distance(x_, y_, target.x, target.y);

            if (d < best_distance) {
                best_distance = d;
                nearest = target;
                found = true;
            }
        }

        return found;
    }

    void controlLoop()
    {
        if (!has_odom_) {
            return;
        }

        double goal_x;
        double goal_y;

        bool going_to_station = false;

        if (current_capacity_ >= max_capacity_) {
            goal_x = station_x_;
            goal_y = station_y_;
            going_to_station = true;
        } else {
            Target nearest;

            if (!findNearestTrash(nearest)) {
                publishStop();
                return;
            }

            goal_x = nearest.x;
            goal_y = nearest.y;
            current_target_id_ = nearest.id;
        }

        double d = distance(x_, y_, goal_x, goal_y);

        if (d <= target_distance_) {
            if (going_to_station) {
                current_capacity_ = 0;
                RCLCPP_INFO(this->get_logger(), "Bot unloaded trash at station");
            } else {
                current_capacity_++;

                RCLCPP_INFO(
                    this->get_logger(),
                    "Bot reached trash. Local capacity: %d/%d",
                    current_capacity_,
                    max_capacity_
                );
            }

            publishStop();
            return;
        }

        double desired_angle = std::atan2(goal_y - y_, goal_x - x_);
        double angle_error = normalizeAngle(desired_angle - theta_);

        geometry_msgs::msg::Twist cmd;

        cmd.angular.z = angular_gain_ * angle_error;

        if (std::fabs(angle_error) < angle_tolerance_) {
            cmd.linear.x = linear_speed_;
        } else {
            cmd.linear.x = 0.0;
        }

        if (cmd.angular.z > 2.0) {
            cmd.angular.z = 2.0;
        }

        if (cmd.angular.z < -2.0) {
            cmd.angular.z = -2.0;
        }

        cmd_pub_->publish(cmd);
    }

    void publishStop()
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);
    }

    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;
    bool has_odom_ = false;

    int max_capacity_ = 0;
    int current_capacity_ = 0;
    int current_target_id_ = -1;

    double station_x_ = 0.0;
    double station_y_ = 0.0;
    double target_distance_ = 0.0;
    double linear_speed_ = 0.0;
    double angular_gain_ = 0.0;
    double angle_tolerance_ = 0.0;

    std::map<int, Target> targets_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<zadanie2_interfaces::msg::GameState>::SharedPtr game_state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BotNode>());
    rclcpp::shutdown();
    return 0;
}