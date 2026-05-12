/*
1. odoberá cmd_vel
2. posiela príkazy do triedy Robot
3. publikuje odometriu a TF
*/

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
        : Node("robot_node") // vytvorenie ROS node pre robota
    {
        declareRequiredParameters(); // deklaracia vsetkych parametrov, ktore musi node dostat z yaml

        ghost_mode_ = getRequiredParameter<bool>("ghost_mode"); // ci robot ignoruje kolizie, player2/bot je ghost
        robot_radius_ = getRequiredParameter<double>("robot_radius"); // polomer robota pouzivany pri koliziach s prekazkami

        environment::Config env_config; // konfiguracia prostredia zo zadania 1
        env_config.map_filename = getRequiredParameter<std::string>("map_path"); // cesta k mape
        env_config.resolution = getRequiredParameter<double>("map_resolution"); // rozlisenie mapy

        env_ = std::make_shared<environment::Environment>(env_config); // vytvorenie prostredia, ktore vie kontrolovat obsadene miesta

        loadObstaclesFromParameters(); // nacitanie kruhovych a obdlznikovych prekazok z yaml

        robot::Robot::CollisionCb collision_cb = nullptr; // callback pre kolizie, posiela sa do triedy Robot zo zadania 1

        if (!ghost_mode_) { // ak robot nie je duch, kontroluje kolizie
            collision_cb = [this](geometry::RobotState state) {
                return isOccupiedByMapOrObstacle(state.x, state.y); // kontrola mapy + geometrickych prekazok
            };
        } else { // ak je ghost_mode true, robot prechadza cez prekazky
            collision_cb = [](geometry::RobotState) {
                return false; // nikdy nenahlasi koliziu
            };
        }

        robot::Config robot_config; // konfiguracia pre triedu Robot zo zadania 1
        robot_config.accelerations.linear =
            getRequiredParameter<double>("linear_acceleration"); // linearne zrychlenie robota
        robot_config.accelerations.angular =
            getRequiredParameter<double>("angular_acceleration"); // uhlove zrychlenie robota

        robot_config.emergency_decelerations.linear =
            getRequiredParameter<double>("linear_emergency_deceleration"); // linearne brzdenie po vyprsani prikazu
        robot_config.emergency_decelerations.angular =
            getRequiredParameter<double>("angular_emergency_deceleration"); // uhlove brzdenie po vyprsani prikazu

        robot_config.command_duration =
            getRequiredParameter<double>("command_duration"); // ako dlho plati posledny cmd_vel prikaz

        robot_config.simulation_period_ms =
            getRequiredParameter<int>("simulation_period_ms"); // perioda simulacie robota v ms

        robot_ = std::make_unique<robot::Robot>(robot_config, collision_cb, true); // vytvorenie robota zo zadania 1, spusti sa jeho vlakno

        geometry::RobotState initial_state; // pociatocny stav robota
        initial_state.x = getRequiredParameter<double>("initial_x"); // pociatocna x pozicia
        initial_state.y = getRequiredParameter<double>("initial_y"); // pociatocna y pozicia
        initial_state.theta = getRequiredParameter<double>("initial_theta"); // pociatocne natocenie
        initial_state.velocity = {0.0, 0.0}; // na zaciatku robot stoji

        if (!ghost_mode_ && isOccupiedByMapOrObstacle(initial_state.x, initial_state.y)) { // ak je spawn v stene alebo prekazke
            RCLCPP_WARN(
                this->get_logger(),
                "Initial position %.2f %.2f is occupied. Searching nearest free position...",
                initial_state.x,
                initial_state.y
            );

            initial_state = findNearestFreeState(initial_state); // najdeme najblizsiu volnu poziciu

            RCLCPP_WARN(
                this->get_logger(),
                "Robot moved to nearest free position %.2f %.2f",
                initial_state.x,
                initial_state.y
            );
        }

        robot_->setState(initial_state); // nastavime pociatocny stav do triedy Robot

        std::string ns = this->get_namespace(); // ziskame namespace, napr /player1 alebo /player2

        if (!ns.empty() && ns[0] == '/') {
            ns.erase(0, 1); // odstranime lomitko na zaciatku namespace
        }

        if (ns.empty()) {
            base_frame_id_ = "base_link"; // ak nema namespace, frame je base_link
        } else {
            base_frame_id_ = ns + "/base_link"; // napr player1/base_link alebo player2/base_link
        }

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this); // broadcaster pre transformaciu map -> base_link

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", // odobera rychlostny prikaz, v namespace napr /player1/cmd_vel
            10,
            std::bind(&RobotNode::cmdVelCallback, this, std::placeholders::_1)
        );

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "odom", // publikuje stav robota, v namespace napr /player1/odom
            10
        );

        int publish_period_ms = getRequiredParameter<int>("publish_period_ms"); // ako casto publikovat odometriu

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(publish_period_ms), // timer na pravidelne publikovanie stavu
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
    struct CircleObstacle // struktura pre kruhovu prekazku
    {
        double x;
        double y;
        double radius;
    };

    struct RectangleObstacle // struktura pre obdlznikovu prekazku
    {
        double x;
        double y;
        double width;
        double height;
    };

    void declareRequiredParameters() // deklaracia vsetkych parametrov, ktore sa nacitaju z yaml
    {
        this->declare_parameter<std::string>("map_path"); // cesta k png mape
        this->declare_parameter<double>("map_resolution"); // rozlisenie mapy

        this->declare_parameter<double>("initial_x"); // pociatocna x pozicia
        this->declare_parameter<double>("initial_y"); // pociatocna y pozicia
        this->declare_parameter<double>("initial_theta"); // pociatocne natocenie

        this->declare_parameter<bool>("ghost_mode"); // ci ma robot ignorovat kolizie
        this->declare_parameter<double>("robot_radius"); // polomer robota

        this->declare_parameter<double>("linear_acceleration"); // linearne zrychlenie
        this->declare_parameter<double>("angular_acceleration"); // uhlove zrychlenie
        this->declare_parameter<double>("linear_emergency_deceleration"); // linearne brzdenie
        this->declare_parameter<double>("angular_emergency_deceleration"); // uhlove brzdenie
        this->declare_parameter<double>("command_duration"); // platnost cmd_vel prikazu
        this->declare_parameter<int>("simulation_period_ms"); // perioda simulacie
        this->declare_parameter<int>("publish_period_ms"); // perioda publikovania odometrie

        this->declare_parameter<std::vector<double>>("circle_obstacles_x"); // x suradnice kruhovych prekazok
        this->declare_parameter<std::vector<double>>("circle_obstacles_y"); // y suradnice kruhovych prekazok
        this->declare_parameter<std::vector<double>>("circle_obstacles_radius"); // polomery kruhovych prekazok

        this->declare_parameter<std::vector<double>>("rectangle_obstacles_x"); // x suradnice obdlznikovych prekazok
        this->declare_parameter<std::vector<double>>("rectangle_obstacles_y"); // y suradnice obdlznikovych prekazok
        this->declare_parameter<std::vector<double>>("rectangle_obstacles_width"); // sirky obdlznikovych prekazok
        this->declare_parameter<std::vector<double>>("rectangle_obstacles_height"); // vysky obdlznikovych prekazok
    }

    template<typename T>
    T getRequiredParameter(const std::string& name) // funkcia na povinne citanie parametrov z yaml
    {
        T value;

        if (!this->get_parameter(name, value)) {
            throw std::runtime_error("Missing required ROS parameter: " + name); // ak parameter chyba, node skonci s chybou
        }

        return value;
    }

    void loadObstaclesFromParameters() // nacitanie geometrickych prekazok z yaml
    {
        auto circle_x = getRequiredParameter<std::vector<double>>("circle_obstacles_x"); // x kruhov
        auto circle_y = getRequiredParameter<std::vector<double>>("circle_obstacles_y"); // y kruhov
        auto circle_r = getRequiredParameter<std::vector<double>>("circle_obstacles_radius"); // radius kruhov

        if (circle_x.size() != circle_y.size() || circle_x.size() != circle_r.size()) {
            throw std::runtime_error("Circle obstacle parameter arrays have different sizes."); // vsetky polia musia mat rovnaku dlzku
        }

        for (size_t i = 0; i < circle_x.size(); ++i) { // vytvorenie vsetkych kruhovych prekazok
            if (circle_r[i] <= 0.0) {
                throw std::runtime_error("Circle obstacle radius must be positive."); // radius musi byt kladny
            }

            circle_obstacles_.push_back({
                circle_x[i],
                circle_y[i],
                circle_r[i]
            });
        }

        auto rect_x = getRequiredParameter<std::vector<double>>("rectangle_obstacles_x"); // x obdlznikov
        auto rect_y = getRequiredParameter<std::vector<double>>("rectangle_obstacles_y"); // y obdlznikov
        auto rect_w = getRequiredParameter<std::vector<double>>("rectangle_obstacles_width"); // sirky obdlznikov
        auto rect_h = getRequiredParameter<std::vector<double>>("rectangle_obstacles_height"); // vysky obdlznikov

        if (rect_x.size() != rect_y.size() ||
            rect_x.size() != rect_w.size() ||
            rect_x.size() != rect_h.size()) {
            throw std::runtime_error("Rectangle obstacle parameter arrays have different sizes."); // vsetky polia musia mat rovnaku dlzku
        }

        for (size_t i = 0; i < rect_x.size(); ++i) { // vytvorenie vsetkych obdlznikovych prekazok
            if (rect_w[i] <= 0.0 || rect_h[i] <= 0.0) {
                throw std::runtime_error("Rectangle obstacle dimensions must be positive."); // sirka a vyska musia byt kladne
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

    bool isInsideCircleObstacle(double x, double y) const // kontrola ci je bod v kruhovej prekazke
    {
        for (const auto& obstacle : circle_obstacles_) {
            double dx = x - obstacle.x;
            double dy = y - obstacle.y;

            if (std::sqrt(dx * dx + dy * dy) <= obstacle.radius + robot_radius_) { // k radiusu pridavame polomer robota
                return true;
            }
        }

        return false;
    }

    bool isInsideRectangleObstacle(double x, double y) const // kontrola ci je bod v obdlznikovej prekazke
    {
        for (const auto& obstacle : rectangle_obstacles_) {
            bool inside =
                x >= obstacle.x - obstacle.width * 0.5 - robot_radius_ && // lava hrana s rezervou polomeru robota
                x <= obstacle.x + obstacle.width * 0.5 + robot_radius_ && // prava hrana s rezervou
                y >= obstacle.y - obstacle.height * 0.5 - robot_radius_ && // spodna hrana s rezervou
                y <= obstacle.y + obstacle.height * 0.5 + robot_radius_; // horna hrana s rezervou

            if (inside) {
                return true;
            }
        }

        return false;
    }

    bool isOccupiedByMapOrObstacle(double x, double y) const // spolocna kontrola kolizie s mapou aj prekazkami
    {
        if (env_->isOccupied(x, y)) { // kontrola ciernej casti png mapy
            return true;
        }

        if (isInsideCircleObstacle(x, y)) { // kontrola kruhovych prekazok
            return true;
        }

        if (isInsideRectangleObstacle(x, y)) { // kontrola obdlznikovych prekazok
            return true;
        }

        return false;
    }

    geometry::RobotState findNearestFreeState(const geometry::RobotState& original_state) // hladanie najblizsej volnej spawn pozicie
    {
        geometry::RobotState free_state = original_state;

        const double step = env_->getResolution(); // krok hladania podla rozlisenia mapy
        const double max_radius = 5.0; // maximalny radius hladania volnej pozicie

        for (double radius = step; radius <= max_radius; radius += step) { // postupne zvacsujeme okolie hladania
            for (double dx = -radius; dx <= radius; dx += step) {
                for (double dy = -radius; dy <= radius; dy += step) {
                    double x = original_state.x + dx;
                    double y = original_state.y + dy;

                    if (!isOccupiedByMapOrObstacle(x, y)) { // ak je miesto volne
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

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) // zavola sa ked pride novy cmd_vel
    {
        geometry::Twist velocity; // struktura zo zadania 1
        velocity.linear = msg->linear.x; // ROS linearna rychlost sa prevedie na nasu strukturu
        velocity.angular = msg->angular.z; // ROS uhlova rychlost sa prevedie na nasu strukturu

        robot_->setVelocity(velocity); // prikaz sa posle do triedy Robot zo zadania 1
    }

    void publishState() // publikovanie aktualneho stavu robota
    {
        geometry::RobotState state = robot_->getState(); // ziskanie stavu z triedy Robot zo zadania 1

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, state.theta); // prevedenie theta na quaternion pre ROS

        nav_msgs::msg::Odometry odom; // ROS sprava odometrie
        odom.header.stamp = this->get_clock()->now(); // aktualny cas
        odom.header.frame_id = "map"; // odometria je voci frame map
        odom.child_frame_id = base_frame_id_; // child frame robota, napr player1/base_link

        odom.pose.pose.position.x = state.x; // poloha x
        odom.pose.pose.position.y = state.y; // poloha y
        odom.pose.pose.position.z = 0.0;

        odom.pose.pose.orientation.x = q.x(); // orientacia ako quaternion
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        odom.twist.twist.linear.x = state.velocity.linear; // aktualna linearna rychlost
        odom.twist.twist.angular.z = state.velocity.angular; // aktualna uhlova rychlost

        odom_pub_->publish(odom); // publikovanie odometrie na topic odom

        geometry_msgs::msg::TransformStamped transform; // TF transformacia map -> base_link
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = base_frame_id_;

        transform.transform.translation.x = state.x; // poloha v transformacii
        transform.transform.translation.y = state.y;
        transform.transform.translation.z = 0.0;

        transform.transform.rotation.x = q.x(); // orientacia v transformacii
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transform); // odoslanie TF pre RViz a ostatne nody
    }

    std::shared_ptr<environment::Environment> env_; // prostredie/mapa zo zadania 1
    std::unique_ptr<robot::Robot> robot_; // trieda Robot zo zadania 1, nezavisla od ROS

    bool ghost_mode_ = false; // ak je true, robot ignoruje kolizie
    double robot_radius_ = 0.0; // polomer robota pre kolizie

    std::vector<CircleObstacle> circle_obstacles_; // zoznam kruhovych prekazok
    std::vector<RectangleObstacle> rectangle_obstacles_; // zoznam obdlznikovych prekazok

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; // broadcaster TF transformacie
    std::string base_frame_id_; // nazov frame robota

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_; // subscriber na cmd_vel
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_; // publisher odometrie
    rclcpp::TimerBase::SharedPtr timer_; // timer na publikovanie stavu
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv); // inicializacia ROS2
    rclcpp::spin(std::make_shared<RobotNode>()); // spustenie robot node
    rclcpp::shutdown(); // ukoncenie ROS2
    return 0;
}