#include <chrono>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class KeyboardTeleopNode : public rclcpp::Node
{
public:
    KeyboardTeleopNode()
        : Node("keyboard_teleop_node")
    {
        this->declare_parameter<double>("linear_speed", 3.0);
        this->declare_parameter<double>("angular_speed", 1.5);

        linear_speed_ = this->get_parameter("linear_speed").as_double();
        angular_speed_ = this->get_parameter("angular_speed").as_double();

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel",
            10
        );

        setupTerminal();

        timer_ = this->create_wall_timer(
            30ms,
            std::bind(&KeyboardTeleopNode::timerCallback, this)
        );

        std::cout << "\nKeyboard teleop started\n";
        std::cout << "Controls:\n";
        std::cout << "  W - forward\n";
        std::cout << "  S - backward\n";
        std::cout << "  A - turn left\n";
        std::cout << "  D - turn right\n";
        std::cout << "  SPACE - stop\n";
        std::cout << "  ESC - exit\n\n";
    }

    ~KeyboardTeleopNode()
    {
        restoreTerminal();
    }

private:
    void setupTerminal()
    {
        tcgetattr(STDIN_FILENO, &old_terminal_);

        termios new_terminal = old_terminal_;
        new_terminal.c_lflag &= ~(ICANON | ECHO);

        tcsetattr(STDIN_FILENO, TCSANOW, &new_terminal);

        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }

    void restoreTerminal()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_terminal_);
    }

    int readKey()
    {
        unsigned char c;
        int result = read(STDIN_FILENO, &c, 1);

        if (result < 0) {
            return -1;
        }

        return c;
    }

    void timerCallback()
    {
        int key = readKey();

        geometry_msgs::msg::Twist cmd;

        if (key == -1) {
            return;
        }

        if (key == 'w' || key == 'W') {
            cmd.linear.x = linear_speed_;
            cmd.angular.z = 0.0;
        } else if (key == 's' || key == 'S') {
            cmd.linear.x = -linear_speed_;
            cmd.angular.z = 0.0;
        } else if (key == 'a' || key == 'A') {
            cmd.linear.x = 0.0;
            cmd.angular.z = -angular_speed_;
        } else if (key == 'd' || key == 'D') {
            cmd.linear.x = 0.0;
            cmd.angular.z = angular_speed_;
        } else if (key == ' ') {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
        } else if (key == 27) {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            cmd_pub_->publish(cmd);

            rclcpp::shutdown();
            return;
        } else {
            return;
        }

        cmd_pub_->publish(cmd);
    }

    double linear_speed_;
    double angular_speed_;

    termios old_terminal_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardTeleopNode>());
    rclcpp::shutdown();
    return 0;
}