#include <memory>
#include <string>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "environment/Environment.h"

class MapNode : public rclcpp::Node
{
public:
    MapNode()
        : Node("map_node") // vytvorenie ROS node pre mapu
    {
        this->declare_parameter<std::string>("map_path"); // deklaracia parametra z yaml - cesta k png mape
        this->declare_parameter<double>("map_resolution"); // deklaracia parametra z yaml - rozlisenie mapy

        environment::Config config; // konfiguracna struktura pre Environment zo zadania 1
        config.map_filename = getRequiredParameter<std::string>("map_path"); // nacitanie cesty k mape z yaml
        config.resolution = getRequiredParameter<double>("map_resolution"); // nacitanie rozlisenia mapy z yaml

        env_ = std::make_shared<environment::Environment>(config); // vytvorenie prostredia, ktore nacita png mapu

        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "map", // topic /map, na ktory sa publikuje mapa
            rclcpp::QoS(1).transient_local().reliable() // aby RViz dostal mapu aj ked sa pripoji neskor
        );

        publishMap(); // hned po spusteni publikujeme mapu

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), // kazdu sekundu mapu publikujeme znovu
            std::bind(&MapNode::publishMap, this)
        );

        RCLCPP_INFO(
            this->get_logger(),
            "map_node loaded map: %s, resolution: %.3f",
            config.map_filename.c_str(),
            config.resolution
        );
    }

private:
    template<typename T>
    T getRequiredParameter(const std::string& name) // pomocna funkcia na citanie povinnych parametrov z yaml
    {
        T value;
        if (!this->get_parameter(name, value)) {
            throw std::runtime_error("Missing required ROS parameter: " + name); // ak parameter chyba, node skonci s chybou
        }
        return value;
    }

    void publishMap() // vytvorenie a publikovanie mapy ako OccupancyGrid
    {
        const cv::Mat& image = env_->getMap(); // ziskame obrazok mapy z Environment

        nav_msgs::msg::OccupancyGrid msg; // ROS sprava pre 2D occupancy mapu
        msg.header.stamp = this->get_clock()->now(); // cas publikovania spravy
        msg.header.frame_id = "map"; // mapa patri do frame map

        msg.info.resolution = env_->getResolution(); // rozlisenie mapy v metroch na pixel
        msg.info.width = image.cols; // sirka mapy v pixeloch
        msg.info.height = image.rows; // vyska mapy v pixeloch

        msg.info.origin.position.x = 0.0; // zaciatok mapy v x
        msg.info.origin.position.y = 0.0; // zaciatok mapy v y
        msg.info.origin.position.z = 0.0;
        msg.info.origin.orientation.w = 1.0; // mapa nie je otocena

        msg.data.resize(image.cols * image.rows); // pole buniek occupancy gridu

        for (int y = 0; y < image.rows; ++y) { // prechadzame vsetky riadky obrazka
            for (int x = 0; x < image.cols; ++x) { // prechadzame vsetky stlpce obrazka
                unsigned char pixel = image.at<unsigned char>(y, x); // hodnota pixelu na pozicii x,y
                int index = y * image.cols + x; // prevod 2D suradnic na 1D index v poli msg.data

                if (pixel == 0) { // cierny pixel znamena prekazku/stenu
                    msg.data[index] = 100; // 100 v OccupancyGrid znamena obsadene miesto
                } else {
                    msg.data[index] = 0; // 0 znamena volne miesto
                }
            }
        }

        map_pub_->publish(msg); // publikovanie mapy na topic /map
    }

    std::shared_ptr<environment::Environment> env_; // objekt Environment zo zadania 1, drzi nacitanu mapu
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_; // publisher pre mapu
    rclcpp::TimerBase::SharedPtr timer_; // timer na opakovane publikovanie mapy
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv); // inicializacia ROS2
    rclcpp::spin(std::make_shared<MapNode>()); // spustenie node, aby bezali callbacky a timer
    rclcpp::shutdown(); // ukoncenie ROS2
    return 0;
}