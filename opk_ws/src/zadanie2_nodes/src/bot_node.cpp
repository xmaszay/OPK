#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "zadanie2_interfaces/msg/game_state.hpp"

struct Target // ciel 
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
        this->declare_parameter<int>("max_capacity"); // deklaracia parametrov z yaml 
        this->declare_parameter<double>("station_x");
        this->declare_parameter<double>("station_y");
        this->declare_parameter<double>("target_distance");
        this->declare_parameter<double>("linear_speed");
        this->declare_parameter<double>("angular_gain");
        this->declare_parameter<double>("angle_tolerance");

        max_capacity_ = getRequiredParameter<int>("max_capacity"); // pomocou funkcie getRe... dostavame parametre z yaml
        station_x_ = getRequiredParameter<double>("station_x");
        station_y_ = getRequiredParameter<double>("station_y");
        target_distance_ = getRequiredParameter<double>("target_distance");
        linear_speed_ = getRequiredParameter<double>("linear_speed");
        angular_gain_ = getRequiredParameter<double>("angular_gain");
        angle_tolerance_ = getRequiredParameter<double>("angle_tolerance");

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/player2/odom", // odobera tento topic
            10,
            std::bind(&BotNode::odomCallback, this, std::placeholders::_1)
        );

        game_state_sub_ = this->create_subscription<zadanie2_interfaces::msg::GameState>(
            "/game_state", // cita zoznam odpadkov, kapacitu bota, poziciu stanice, pocet zostavaj. odpadkov,.. 
            10,
            std::bind(&BotNode::gameStateCallback, this, std::placeholders::_1)
        );

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/player2/cmd_vel", // bot publikuje prikazy na /player2/cmd...
            10
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // kazdych 100ms sa rozhoduje kam ma ist bot a aky prikaz poslat
            std::bind(&BotNode::controlLoop, this)
        );

        RCLCPP_INFO(this->get_logger(), "bot_node started for player2");
    }

private:
    template<typename T>
    T getRequiredParameter(const std::string& name) // funkcia na citanie parametrov, ak nema hodi vynimku 
    {
        T value;

        if (!this->get_parameter(name, value)) {
            throw std::runtime_error("Missing required ROS parameter: " + name);
        }

        return value;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) // zavola sa vzdy ked pride nova sprava z player2/odom
    {
        x_ = msg->pose.pose.position.x; // ulozenie aktualnej polohy robota
        y_ = msg->pose.pose.position.y;

        const auto& q = msg->pose.pose.orientation; // v ROS je ulozena orientacia v quaternione

        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y); // prepocitame z qua. na thetu pre robota
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        theta_ = std::atan2(siny_cosp, cosy_cosp);

        has_odom_ = true;
    }

    void gameStateCallback(const zadanie2_interfaces::msg::GameState::SharedPtr msg) // zavolame funkciu vzdy ked pride novy stav hry
    {
        targets_.clear();

        current_capacity_ = msg->player2_capacity; // citanie stavu z game_node
        remaining_trash_ = msg->remaining_trash;
        game_finished_ = msg->game_finished;

        station_x_ = msg->station_x;
        station_y_ = msg->station_y;

        for (size_t i = 0; i < msg->trash_id.size(); ++i) { // for cez vsetky odpadky
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

            targets_[target.id] = target; // kazdy odpad ma svoje ID
        }

        has_game_state_ = true;
    }

    double normalizeAngle(double angle) const // funkcia pre otacanie bota, aby sa otacal kratsim smerom k odpadkom
    {
        while (angle > M_PI) {
            angle -= 2.0 * M_PI;
        }

        while (angle < -M_PI) {
            angle += 2.0 * M_PI;
        }

        return angle;
    }

    double distance(double x1, double y1, double x2, double y2) const // pocita euklidovsku vzdialenost 2 bodov
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
        double best_distance = 1e9; // nastavene na velke cislo, aby sa najblizsi odpad vedel definovat 

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

    void controlLoop() // publikujeme ju kazdych 100ms 
    {
        if (!has_odom_ || !has_game_state_) {
            return;
        }

        if (game_finished_) {
            publishStop();
            return;
        }

        double goal_x = 0.0;
        double goal_y = 0.0;
        bool going_to_station = false;

        /*
         * Dolezite:
         * Kapacita bota sa uz nepocita lokalne.
         * Autoritou je game_node cez /game_state -> player2_capacity.
         *
         * Ak bot nieco nesie a uz nie su ziadne odpadky,
         * musi ist do stanice, inak sa nikdy nevypise vitaz.
         */

        if (current_capacity_ >= max_capacity_) { // ak je plna kapacita, posleme ho do station
            goal_x = station_x_;
            goal_y = station_y_;
            going_to_station = true;
        } else if (remaining_trash_ == 0 && current_capacity_ > 0) {
            goal_x = station_x_;
            goal_y = station_y_;
            going_to_station = true;
        } else {
            Target nearest;

            if (!findNearestTrash(nearest)) {
                if (current_capacity_ > 0) {
                    goal_x = station_x_;
                    goal_y = station_y_;
                    going_to_station = true;
                } else {
                    publishStop();
                    return;
                }
            } else {
                goal_x = nearest.x;
                goal_y = nearest.y;
                current_target_id_ = nearest.id;
            }
        }

        double d = distance(x_, y_, goal_x, goal_y);

        if (d <= target_distance_) {
            /*
             * Tu uz nemenime current_capacity_.
             * O zbere a vylozeni rozhoduje game_node.
             * Bot iba dojde k cielu a zastavi.
             */

            if (going_to_station) {
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Bot is at station, waiting for game_node to unload."
                );
            } else {
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Bot reached trash, waiting for game_node to collect."
                );
            }

            publishStop();
            return;
        }

        double desired_angle = std::atan2(goal_y - y_, goal_x - x_); // uhol ktorym smerom sa nachadza ciel
        double angle_error = normalizeAngle(desired_angle - theta_); // rozdiel medzi desire a aktualnym natocenim robota

        geometry_msgs::msg::Twist cmd;

        cmd.angular.z = angular_gain_ * angle_error; // bot sa otaca umerne chybe - ak je velka chyba otaca sa rychlejsie

        if (std::fabs(angle_error) < angle_tolerance_) { // ak je bot natoceny inde ako k smeru k cielu, najprv sa otoci
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

    void publishStop() // posiela nulove pohybove prikazy 
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
    bool has_game_state_ = false;
    bool game_finished_ = false;

    int max_capacity_ = 0;
    int current_capacity_ = 0;
    int remaining_trash_ = 0;
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