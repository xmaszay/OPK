#include <chrono>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class KeyboardTeleopNode : public rclcpp::Node
{
public:
    KeyboardTeleopNode()
        : Node("keyboard_teleop_node") // vytvorenie ROS node pre ovladanie klavesnicou
    {
        this->declare_parameter<double>("linear_speed"); // deklaracia linearnej rychlosti z yaml
        this->declare_parameter<double>("angular_speed"); // deklaracia uhlovej rychlosti z yaml

        linear_speed_ = getRequiredParameter<double>("linear_speed"); // nacitanie linearnej rychlosti z yaml
        angular_speed_ = getRequiredParameter<double>("angular_speed"); // nacitanie uhlovej rychlosti z yaml

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", // publikuje prikazy na cmd_vel, pri spusteni sa remapuje na /player1/cmd_vel
            10
        );

        setupTerminal(); // nastavenie terminalu aby sme vedeli citat klavesy bez enteru

        timer_ = this->create_wall_timer(
            30ms, // kazdych 30ms kontrolujeme ci bola stlacena klavesa
            std::bind(&KeyboardTeleopNode::timerCallback, this)
        );

        std::cout << "\nKeyboard teleop started\n";
        std::cout << "Controls:\n";
        std::cout << "  W - forward\n";
        std::cout << "  S - backward\n";
        std::cout << "  A - turn left\n";
        std::cout << "  D - turn right\n";
        std::cout << "  ESC - exit\n\n";
    }

    ~KeyboardTeleopNode()
    {
        restoreTerminal(); // pri ukonceni vratime terminal do povodneho stavu
    }

private:
    template<typename T>
    T getRequiredParameter(const std::string& name) // funkcia na povinne citanie parametrov z yaml
    {
        T value;
        if (!this->get_parameter(name, value)) {
            throw std::runtime_error("Missing required ROS parameter: " + name); // ak parameter chyba, hodi vynimku
        }
        return value;
    }

    void setupTerminal() // nastavenie terminalu pre okamzite citanie klaves
    {
        tcgetattr(STDIN_FILENO, &old_terminal_); // ulozenie povodneho nastavenia terminalu

        termios new_terminal = old_terminal_; // vytvorime kopiu povodnych nastaveni
        new_terminal.c_lflag &= ~(ICANON | ECHO); // vypne kanonicky mod a echo, netreba stlacat enter a nevypisuje znaky

        tcsetattr(STDIN_FILENO, TCSANOW, &new_terminal); // nastavi novy terminal hned

        int flags = fcntl(STDIN_FILENO, F_GETFL, 0); // ziskame aktualne flagy stdin
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK); // nastavime neblokujuci vstup, aby program necakal na klavesu
    }

    void restoreTerminal() // vratenie terminalu do povodneho stavu
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_terminal_);
    }

    int readKey() // precita jednu klavesu zo stdin
    {
        unsigned char c;
        int result = read(STDIN_FILENO, &c, 1); // pokus o precitanie jedneho znaku

        if (result < 0) {
            return -1; // ak nebola stlacena klavesa, vratime -1
        }

        return c; // vrati ascii kod stlacenej klavesy
    }

    void timerCallback() // funkcia volana timerom, spracuje klavesu a posle cmd_vel
    {
        int key = readKey(); // precitanie stlacenej klavesy

        geometry_msgs::msg::Twist cmd; // sprava s rychlostnym prikazom pre robota

        if (key == -1) {
            return; // ak nebola stlacena ziadna klavesa, nic neposielame
        }

        if (key == 'w' || key == 'W') { // pohyb dopredu
            cmd.linear.x = linear_speed_;
            cmd.angular.z = 0.0;
        } else if (key == 's' || key == 'S') { // pohyb dozadu
            cmd.linear.x = -linear_speed_;
            cmd.angular.z = 0.0;
        } else if (key == 'd' || key == 'D') { // otacanie dolava
            cmd.linear.x = 0.0;
            cmd.angular.z = -angular_speed_;
        } else if (key == 'a' || key == 'A') { // otacanie doprava
            cmd.linear.x = 0.0;
            cmd.angular.z = angular_speed_;
        } else if (key == ' ') { // stop
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
        } else if (key == 27) { // ESC ukonci teleop
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            cmd_pub_->publish(cmd); // pred ukoncenim posleme stop prikaz

            rclcpp::shutdown(); // ukoncenie ROS node
            return;
        } else {
            return; // ine klavesy ignorujeme
        }

        cmd_pub_->publish(cmd); // publikovanie rychlostneho prikazu robotovi
    }

    double linear_speed_ = 0.0; // linearna rychlost nacitana z yaml
    double angular_speed_ = 0.0; // uhlova rychlost nacitana z yaml

    termios old_terminal_; // povodne nastavenie terminalu, aby sme ho vedeli obnovit

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_; // publisher pre cmd_vel
    rclcpp::TimerBase::SharedPtr timer_; // timer na periodicke citanie klaves
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv); // inicializacia ROS2
    rclcpp::spin(std::make_shared<KeyboardTeleopNode>()); // spustenie node a cakanie na callbacky
    rclcpp::shutdown(); // ukoncenie ROS2
    return 0;
}