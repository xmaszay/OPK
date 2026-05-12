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
        : Node("lidar_node") // vytvorenie ROS node pre lidar
    {
        this->declare_parameter<std::string>("map_path"); // cesta k png mape z yaml
        this->declare_parameter<double>("map_resolution"); // rozlisenie mapy z yaml
        this->declare_parameter<std::string>("base_frame_id"); // frame robota, v ktorom bude publikovany scan

        this->declare_parameter<double>("max_range"); // maximalny dosah lidaru
        this->declare_parameter<int>("beam_count"); // pocet lucov lidaru
        this->declare_parameter<double>("first_ray_angle"); // prvy uhol skenu
        this->declare_parameter<double>("last_ray_angle"); // posledny uhol skenu
        this->declare_parameter<int>("publish_period_ms"); // perioda publikovania skenu
        this->declare_parameter<double>("ray_step"); // krok raytracingu, cim vacsi krok tym rychlejsi vypocet

        environment::Config env_config; // konfiguracia prostredia zo zadania 1
        env_config.map_filename = getRequiredParameter<std::string>("map_path"); // nacitanie cesty k mape
        env_config.resolution = getRequiredParameter<double>("map_resolution"); // nacitanie rozlisenia mapy

        env_ = std::make_shared<environment::Environment>(env_config); // vytvorenie mapy/prostredia, pouziva sa na kontrolu prekazok

        base_frame_id_ = getRequiredParameter<std::string>("base_frame_id"); // nacitanie frame id pre LaserScan
        max_range_ = getRequiredParameter<double>("max_range"); // nacitanie max dosahu
        beam_count_ = getRequiredParameter<int>("beam_count"); // nacitanie poctu lucov
        first_ray_angle_ = getRequiredParameter<double>("first_ray_angle"); // nacitanie zaciatocneho uhla
        last_ray_angle_ = getRequiredParameter<double>("last_ray_angle"); // nacitanie koncoveho uhla
        ray_step_ = getRequiredParameter<double>("ray_step"); // nacitanie kroku luca z yaml

        if (ray_step_ <= 0.0) {
            throw std::runtime_error("ray_step must be positive.");
        }

        if (beam_count_ <= 0) {
            throw std::runtime_error("beam_count must be positive.");
        }

        if (max_range_ <= 0.0) {
            throw std::runtime_error("max_range must be positive.");
        }

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", // odobera odometriu robota, pri namespace je to napr /player1/odom alebo /player2/odom
            10,
            std::bind(&LidarNode::odomCallback, this, std::placeholders::_1)
        );

        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "scan", // publikuje 2D lidar scan, v namespace napr /player1/scan
            10
        );

        int publish_period_ms = getRequiredParameter<int>("publish_period_ms"); // ako casto sa ma publikovat scan

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(publish_period_ms), // timer podla parametra z yaml
            std::bind(&LidarNode::publishScan, this)
        );

        RCLCPP_INFO(
            this->get_logger(),
            "lidar_node started. base_frame: %s, max_range: %.2f, beam_count: %d, ray_step: %.3f",
            base_frame_id_.c_str(),
            max_range_,
            beam_count_,
            ray_step_
        );
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

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) // zavola sa vzdy ked pride nova odometria robota
    {
        robot_x_ = msg->pose.pose.position.x; // ulozenie aktualnej x pozicie robota
        robot_y_ = msg->pose.pose.position.y; // ulozenie aktualnej y pozicie robota

        const auto& q = msg->pose.pose.orientation; // orientacia je v ROS ulozena ako quaternion
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y); // prepocet quaternionu na yaw/theta
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        robot_theta_ = std::atan2(siny_cosp, cosy_cosp); // aktualne natocenie robota

        has_odom_ = true; // uz mame platnu odometriu, mozeme pocitat scan
    }

    double traceSingleRay(double world_angle) const // vystreli jeden luc a vrati vzdialenost k najblizsej prekazke
    {
        for (double distance = 0.0; distance <= max_range_; distance += ray_step_) { // postupne ideme po luci od robota az po max range
            double x = robot_x_ + distance * std::cos(world_angle); // bod na luci v x
            double y = robot_y_ + distance * std::sin(world_angle); // bod na luci v y

            if (env_->isOccupied(x, y)) { // kontrola ci bod narazil do steny/mapy
                return distance; // vratime vzdialenost narazu
            }
        }

        return std::numeric_limits<float>::infinity(); // ak nic netrafilo, dame inf, v RViz sa taky bod nekresli
    }

    void publishScan() // vytvori a publikuje LaserScan spravu
    {
        if (!has_odom_) { // kym nemame polohu robota, nevieme pocitat scan
            return;
        }

        auto start_time = std::chrono::steady_clock::now(); // zaciatok merania casu vypoctu skenu

        sensor_msgs::msg::LaserScan scan; // ROS sprava pre 2D lidar
        scan.header.stamp = this->get_clock()->now(); // aktualny cas spravy
        scan.header.frame_id = base_frame_id_; // frame lidaru/robota

        scan.angle_min = first_ray_angle_; // zaciatocny uhol skenu
        scan.angle_max = last_ray_angle_; // koncovy uhol skenu
        scan.range_min = 0.0; // minimalna meratelna vzdialenost
        scan.range_max = max_range_; // maximalna meratelna vzdialenost
        scan.time_increment = 0.0; // cas medzi lucmi neriesime
        scan.scan_time = 0.1; // priblizny cas jedneho skenu

        if (beam_count_ <= 1) { // specialny pripad, ak mame iba jeden luc
            scan.angle_increment = 0.0;
            scan.ranges.resize(1);

            double world_angle = robot_theta_ + first_ray_angle_; // svetovy uhol luca
            scan.ranges[0] = static_cast<float>(traceSingleRay(world_angle)); // vypocet vzdialenosti pre jeden luc
        } else {
            scan.angle_increment =
                (last_ray_angle_ - first_ray_angle_) / static_cast<double>(beam_count_ - 1); // rozdelenie uhloveho rozsahu medzi luce

            scan.ranges.resize(beam_count_); // pripravime pole vzdialenosti pre vsetky luce

            for (int i = 0; i < beam_count_; ++i) { // prechadzame vsetky luce
                double relative_angle = first_ray_angle_ + i * scan.angle_increment; // uhol luca voci robotovi
                double world_angle = robot_theta_ + relative_angle; // uhol luca vo svete

                scan.ranges[i] = static_cast<float>(traceSingleRay(world_angle)); // vypocet vzdialenosti narazu pre dany luc
            }
        }

        auto end_time = std::chrono::steady_clock::now(); // koniec merania casu vypoctu
        double elapsed_ms =
            std::chrono::duration<double, std::milli>(end_time - start_time).count(); // cas vypoctu skenu v ms

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Lidar scan computation time: %.2f ms, ray_step: %.3f, beams: %d",
            elapsed_ms,
            ray_step_,
            beam_count_
        ); // vypis casu vypoctu kazde 2 sekundy, aby konzola nebola zahltena

        scan_pub_->publish(scan); // publikovanie skenu na topic scan
    }

    std::shared_ptr<environment::Environment> env_; // prostredie/mapa zo zadania 1

    double robot_x_ = 0.0; // aktualna x pozicia robota
    double robot_y_ = 0.0; // aktualna y pozicia robota
    double robot_theta_ = 0.0; // aktualne natocenie robota
    bool has_odom_ = false; // informacia ci uz prisla odometria

    std::string base_frame_id_; // frame id pre LaserScan
    double max_range_ = 0.0; // maximalny dosah lidaru
    int beam_count_ = 0; // pocet lucov
    double first_ray_angle_ = 0.0; // prvy uhol lidaru
    double last_ray_angle_ = 0.0; // posledny uhol lidaru
    double ray_step_ = 0.02; // krok luca pri raytracingu

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; // subscriber na odometriu
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_; // publisher na LaserScan
    rclcpp::TimerBase::SharedPtr timer_; // timer na periodicke publikovanie skenu
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv); // inicializacia ROS2
    rclcpp::spin(std::make_shared<LidarNode>()); // spustenie lidar node
    rclcpp::shutdown(); // ukoncenie ROS2
    return 0;
}