#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

#include <chrono>

class TeleopNode : public rclcpp::Node
{
public:
    TeleopNode() : Node("teleop_node")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        linear_speed_ = 1;
        angular_speed_ = 1;

        current_linear_ = 0.0;
        current_angular_ = 0.0;
        key_active_ = false;
        last_key_time_ = this->now();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&TeleopNode::update, this));

        RCLCPP_INFO(this->get_logger(), "Teleop node started.");
        RCLCPP_INFO(this->get_logger(), "Controls: W/S forward-backward, A/D left-right, Q quit");

        setupTerminal();
    }

    ~TeleopNode()
    {
        restoreTerminal();
    }

private:
    void setupTerminal()
    {
        tcgetattr(STDIN_FILENO, &old_tio_);
        termios new_tio = old_tio_;

        new_tio.c_lflag &= ~(ICANON | ECHO);
        new_tio.c_cc[VMIN] = 0;
        new_tio.c_cc[VTIME] = 0;

        tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
    }

    void restoreTerminal()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
    }

    bool readKey(char &c)
    {
        fd_set set;
        struct timeval timeout;

        FD_ZERO(&set);
        FD_SET(STDIN_FILENO, &set);

        timeout.tv_sec = 0;
        timeout.tv_usec = 0;

        int rv = select(STDIN_FILENO + 1, &set, nullptr, nullptr, &timeout);
        if (rv > 0) {
            return read(STDIN_FILENO, &c, 1) > 0;
        }

        return false;
    }
void publishCommand(double linear, double angular)
{
    geometry_msgs::msg::Twist msg;
    msg.linear.x = linear;
    msg.angular.z = angular;
    publisher_->publish(msg);

    RCLCPP_INFO(this->get_logger(),
                "Published cmd: linear=%.2f angular=%.2f",
                linear, angular);
}

    void update()
    {
        char key;
        bool got_key = false;

        while (readKey(key)) {
            got_key = true;

            switch (key) {
                case 'w':
                case 'W':
                    current_linear_ = linear_speed_;
                    current_angular_ = 0.0;
                    key_active_ = true;
                    last_key_time_ = this->now();
                    break;

                case 's':
                case 'S':
                    current_linear_ = -linear_speed_;
                    current_angular_ = 0.0;
                    key_active_ = true;
                    last_key_time_ = this->now();
                    break;

                case 'a':
                case 'A':
                    current_linear_ = 0.0;
                    current_angular_ = angular_speed_;
                    key_active_ = true;
                    last_key_time_ = this->now();
                    break;

                case 'd':
                case 'D':
                    current_linear_ = 0.0;
                    current_angular_ = -angular_speed_;
                    key_active_ = true;
                    last_key_time_ = this->now();
                    break;

                case 'q':
                case 'Q':
                    publishCommand(0.0, 0.0);
                    RCLCPP_INFO(this->get_logger(), "Quitting teleop node.");
                    rclcpp::shutdown();
                    return;

                default:
                    break;
            }
        }

        (void)got_key;

        double elapsed = (this->now() - last_key_time_).seconds();

        if (key_active_ && elapsed > 0.15) {
            current_linear_ = 0.0;
            current_angular_ = 0.0;
            key_active_ = false;
        }

        publishCommand(current_linear_, current_angular_);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    termios old_tio_{};

    double linear_speed_;
    double angular_speed_;
    double current_linear_;
    double current_angular_;
    bool key_active_;
    rclcpp::Time last_key_time_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopNode>());
    rclcpp::shutdown();
    return 0;
}