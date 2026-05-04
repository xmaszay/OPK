#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

#include "robot/Robot.h"
#include "types/Geometry.h"

#include <sstream>
#include <memory>

class RobotNode : public rclcpp::Node
{
public:
    RobotNode()
        : Node("robot_node"),
          robot_(createRobotConfig())
    {
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(&RobotNode::cmdCallback, this, std::placeholders::_1));

        state_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/robot_state",
            10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RobotNode::publishState, this));

        RCLCPP_INFO(this->get_logger(), "Robot node started.");
    }

private:
    static robot::Config createRobotConfig()
    {
        robot::Config cfg;
        cfg.accelerations.linear = 0.5;
        cfg.accelerations.angular = 1.0;
        cfg.emergency_decelerations.linear = 1.0;
        cfg.emergency_decelerations.angular = 2.0;
        cfg.command_duration = 0.5;
        cfg.simulation_period_ms = 50;
        return cfg;
    }

    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        geometry::Twist cmd;
        cmd.linear = msg->linear.x;
        cmd.angular = msg->angular.z;

        robot_.setVelocity(cmd);
    }

void publishState()
{
    auto state = robot_.getState();
    bool collision = robot_.isInCollision();

    std_msgs::msg::String msg;
    std::ostringstream oss;

    oss << "x=" << state.x
        << ", y=" << state.y
        << ", theta=" << state.theta
        << ", v=" << state.velocity.linear
        << ", w=" << state.velocity.angular
        << ", collision=" << (collision ? "true" : "false");

    msg.data = oss.str();
    state_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "%s", msg.data.c_str());
}

    robot::Robot robot_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotNode>());
    rclcpp::shutdown();
    return 0;
}