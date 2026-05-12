#include <memory>
#include <string>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "zadanie2_interfaces/msg/game_state.hpp"

struct RobotVisualState // pomocna struktura na ulozenie vizualneho stavu robota
{
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    bool has_odom = false; // ci uz prisla odometria robota
};

struct LidarVisualState // pomocna struktura na ulozenie stavu lidaru
{
    std::vector<float> ranges; // vzdialenosti jednotlivych lucov
    double angle_min = 0.0; // zaciatocny uhol skenu
    double angle_increment = 0.0; // rozdiel uhla medzi lucmi
    double range_min = 0.0; // minimalny rozsah
    double range_max = 0.0; // maximalny rozsah
    bool has_scan = false; // ci uz prisiel scan
};

class VisualizationNode : public rclcpp::Node
{
public:
    VisualizationNode()
        : Node("visualization_node") // vytvorenie ROS node pre vizualizaciu
    {
        player1_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/player1/odom", // odobera odometriu prveho robota
            10,
            std::bind(&VisualizationNode::player1OdomCallback, this, std::placeholders::_1)
        );

        player2_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/player2/odom", // odobera odometriu druheho robota/bota
            10,
            std::bind(&VisualizationNode::player2OdomCallback, this, std::placeholders::_1)
        );

        player1_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/player1/scan", // odobera lidar scan prveho robota
            10,
            std::bind(&VisualizationNode::player1ScanCallback, this, std::placeholders::_1)
        );

        player2_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/player2/scan", // odobera lidar scan druheho robota
            10,
            std::bind(&VisualizationNode::player2ScanCallback, this, std::placeholders::_1)
        );

        game_state_sub_ = this->create_subscription<zadanie2_interfaces::msg::GameState>(
            "/game_state", // odobera stav hry - odpadky, stanica, prekazky, skore, drahy
            10,
            std::bind(&VisualizationNode::gameStateCallback, this, std::placeholders::_1)
        );

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "visualization_markers", // publikuje vsetky markery pre RViz
            10
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // kazdych 100ms znovu vykresli markery
            std::bind(&VisualizationNode::publishMarkers, this)
        );

        RCLCPP_INFO(this->get_logger(), "visualization_node started");
    }

private:
    void player1OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) // callback pre odometriu hraca 1
    {
        updateRobotState(player1_, msg); // aktualizuje ulozeny stav player1
    }

    void player2OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) // callback pre odometriu hraca 2
    {
        updateRobotState(player2_, msg); // aktualizuje ulozeny stav player2
    }

    void player1ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) // callback pre lidar hraca 1
    {
        updateLidarState(player1_lidar_, msg); // ulozi lidar data hraca 1
    }

    void player2ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) // callback pre lidar hraca 2
    {
        updateLidarState(player2_lidar_, msg); // ulozi lidar data hraca 2
    }

    void updateRobotState(
        RobotVisualState& state,
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        state.x = msg->pose.pose.position.x; // ulozenie x pozicie robota
        state.y = msg->pose.pose.position.y; // ulozenie y pozicie robota

        const auto& q = msg->pose.pose.orientation; // orientacia je v ROS ako quaternion
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y); // prepocet quaternionu na theta/yaw
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        state.theta = std::atan2(siny_cosp, cosy_cosp); // ulozenie natocenia robota

        state.has_odom = true; // mame platnu odometriu
    }

    void updateLidarState(
        LidarVisualState& state,
        const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        state.ranges = msg->ranges; // ulozenie vzdialenosti lucov
        state.angle_min = msg->angle_min; // ulozenie zaciatocneho uhla
        state.angle_increment = msg->angle_increment; // ulozenie uhloveho kroku
        state.range_min = msg->range_min; // ulozenie min rozsahu
        state.range_max = msg->range_max; // ulozenie max rozsahu
        state.has_scan = true; // mame platny scan
    }

    void gameStateCallback(const zadanie2_interfaces::msg::GameState::SharedPtr msg) // callback pre stav hry
    {
        last_game_state_ = *msg; // ulozenie posledneho stavu hry
        has_game_state_ = true; // uz mame stav hry
    }

    visualization_msgs::msg::Marker createRobotMarker(
        const RobotVisualState& robot,
        int id,
        const std::string& ns,
        float r,
        float g,
        float b,
        float a)
    {
        visualization_msgs::msg::Marker marker; // marker pre vykreslenie robota
        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = "map"; // robot sa vykresluje vo frame map

        marker.ns = ns; // namespace markera, napr player1_robot
        marker.id = id; // id markera
        marker.type = visualization_msgs::msg::Marker::CYLINDER; // robot je vykresleny ako valec
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = robot.x; // poloha robota x
        marker.pose.position.y = robot.y; // poloha robota y
        marker.pose.position.z = 0.10;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.7; // priemer robota v x
        marker.scale.y = 0.7; // priemer robota v y
        marker.scale.z = 0.2; // vyska valca

        marker.color.r = r; // farba robota
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = a; // priehladnost

        return marker;
    }

    visualization_msgs::msg::Marker createLidarHitsMarker(
        const RobotVisualState& robot,
        const LidarVisualState& lidar,
        int id,
        const std::string& ns,
        float r,
        float g,
        float b)
    {
        visualization_msgs::msg::Marker marker; // marker pre body narazov lidaru
        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = "map";

        marker.ns = ns; // namespace napr player1_lidar_hits
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::POINTS; // lidar body kreslime ako body
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.05; // velkost bodov
        marker.scale.y = 0.05;

        marker.color.r = r; // farba lidar bodov
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0f;

        if (!robot.has_odom || !lidar.has_scan) { // ak nemame odometriu alebo scan, nevieme body vypocitat
            return marker;
        }

        for (size_t i = 0; i < lidar.ranges.size(); ++i) { // prechadzame vsetky luce
            double range = static_cast<double>(lidar.ranges[i]); // vzdialenost narazu luca

            if (!std::isfinite(range)) { // ak je inf, luc nic netrafil
                continue;
            }

            if (range < lidar.range_min || range > lidar.range_max) { // ignorovanie neplatnych vzdialenosti
                continue;
            }

            double local_angle =
                lidar.angle_min + static_cast<double>(i) * lidar.angle_increment; // uhol luca voci robotovi
            double world_angle = robot.theta + local_angle; // uhol luca vo svete

            geometry_msgs::msg::Point p;
            p.x = robot.x + range * std::cos(world_angle); // vypocet x bodu narazu
            p.y = robot.y + range * std::sin(world_angle); // vypocet y bodu narazu
            p.z = 0.03;

            marker.points.push_back(p); // pridanie bodu do markera
        }

        return marker;
    }

    visualization_msgs::msg::Marker createTrashMarker(size_t i) // vytvorenie markera pre jeden odpadok
    {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = "map";

        marker.ns = "trash";
        marker.id = last_game_state_.trash_id[i]; // id odpadu pouzijeme ako id markera
        marker.type = visualization_msgs::msg::Marker::SPHERE; // odpad kreslime ako gulicku

        marker.action = last_game_state_.trash_collected[i]
            ? visualization_msgs::msg::Marker::DELETE // ak je pozbierany, marker sa zmaze
            : visualization_msgs::msg::Marker::ADD; // ak nie, marker sa zobrazi

        marker.pose.position.x = last_game_state_.trash_x[i]; // poloha odpadu x
        marker.pose.position.y = last_game_state_.trash_y[i]; // poloha odpadu y
        marker.pose.position.z = last_game_state_.trash_radius[i];
        marker.pose.orientation.w = 1.0;

        marker.scale.x = last_game_state_.trash_radius[i] * 2.0; // priemer odpadu
        marker.scale.y = last_game_state_.trash_radius[i] * 2.0;
        marker.scale.z = last_game_state_.trash_radius[i] * 2.0;

        const std::string& type = last_game_state_.trash_type[i]; // typ odpadu

        if (type == "paper") { // papier ma zltu farbu
            marker.color.r = 1.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.2f;
        } else if (type == "plastic") { // plast ma oranzovu farbu
            marker.color.r = 1.0f;
            marker.color.g = 0.7f;
            marker.color.b = 0.0f;
        } else { // sklo alebo iny typ ma zelenu farbu
            marker.color.r = 0.4f;
            marker.color.g = 1.0f;
            marker.color.b = 0.4f;
        }

        marker.color.a = 1.0f;
        return marker;
    }

    visualization_msgs::msg::Marker createStationMarker() // vytvorenie markera pre stanicu
    {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = "map";

        marker.ns = "station";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CYLINDER; // stanica je valec/kruh
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = last_game_state_.station_x; // poloha stanice x
        marker.pose.position.y = last_game_state_.station_y; // poloha stanice y
        marker.pose.position.z = 0.05;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = last_game_state_.station_radius * 2.0; // priemer stanice
        marker.scale.y = last_game_state_.station_radius * 2.0;
        marker.scale.z = 0.1;

        marker.color.r = 0.0f; // stanica je zelena
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.8f;

        return marker;
    }

    visualization_msgs::msg::Marker createCircleObstacleMarker(size_t i, int id) // marker pre kruhovu prekazku
    {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = "map";

        marker.ns = "obstacles";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::CYLINDER; // kruhova prekazka je valec
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = last_game_state_.circle_obstacles_x[i]; // poloha kruhovej prekazky x
        marker.pose.position.y = last_game_state_.circle_obstacles_y[i]; // poloha kruhovej prekazky y
        marker.pose.position.z = 0.08;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = last_game_state_.circle_obstacles_radius[i] * 2.0; // priemer prekazky
        marker.scale.y = last_game_state_.circle_obstacles_radius[i] * 2.0;
        marker.scale.z = 0.15;

        marker.color.r = 1.0f; // prekazky su cervene
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.7f;

        return marker;
    }

    visualization_msgs::msg::Marker createRectangleObstacleMarker(size_t i, int id) // marker pre obdlznikovu prekazku
    {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = "map";

        marker.ns = "obstacles";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::CUBE; // obdlznikova prekazka je kocka/kvader
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = last_game_state_.rectangle_obstacles_x[i]; // poloha obdlznika x
        marker.pose.position.y = last_game_state_.rectangle_obstacles_y[i]; // poloha obdlznika y
        marker.pose.position.z = 0.08;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = last_game_state_.rectangle_obstacles_width[i]; // sirka obdlznika
        marker.scale.y = last_game_state_.rectangle_obstacles_height[i]; // vyska/dlzka obdlznika v mape
        marker.scale.z = 0.15;

        marker.color.r = 1.0f; // prekazky su cervene
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.7f;

        return marker;
    }

    visualization_msgs::msg::Marker createPathMarker(
        const std::vector<double>& path_x,
        const std::vector<double>& path_y,
        int id,
        const std::string& ns,
        float r,
        float g,
        float b)
    {
        visualization_msgs::msg::Marker marker; // marker pre prejdenu drahu robota
        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = "map";

        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP; // draha sa kresli ako spojena ciara
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.05; // hrubka ciary

        marker.color.r = r; // farba drahy
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0f;

        size_t count = std::min(path_x.size(), path_y.size()); // ochrana proti rozdielnym velkostiam poli

        for (size_t i = 0; i < count; ++i) { // pridame vsetky body drahy
            geometry_msgs::msg::Point p;
            p.x = path_x[i];
            p.y = path_y[i];
            p.z = 0.04;

            marker.points.push_back(p);
        }

        return marker;
    }

    visualization_msgs::msg::Marker createScoreTextMarker() // textovy marker pre skore a stav hry
    {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = "map";

        marker.ns = "score";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING; // text sa otaca k pohladu v RViz
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = 2.0; // poloha textu v mape
        marker.pose.position.y = 1.0;
        marker.pose.position.z = 1.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.z = 0.45; // velkost textu

        marker.color.r = 1.0f; // text je biely
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0f;

        marker.text =
            "DUEL MODE - " + last_game_state_.status + // stav hry
            "\nP1 score: " + std::to_string(last_game_state_.player1_score) +
            " | cap: " + std::to_string(last_game_state_.player1_capacity) +
            "\nP2 score: " + std::to_string(last_game_state_.player2_score) +
            " | cap: " + std::to_string(last_game_state_.player2_capacity) +
            "\nRemaining trash: " + std::to_string(last_game_state_.remaining_trash);

        return marker;
    }

    void publishMarkers() // hlavna funkcia na publikovanie vsetkych markerov do RViz
    {
        visualization_msgs::msg::MarkerArray array; // pole markerov

        if (player1_.has_odom) { // ak mame polohu hraca 1, vykreslime ho
            array.markers.push_back(
                createRobotMarker(player1_, 0, "player1_robot", 0.0f, 0.3f, 1.0f, 1.0f)
            );
        }

        if (player2_.has_odom) { // ak mame polohu hraca 2, vykreslime ho
            array.markers.push_back(
                createRobotMarker(player2_, 0, "player2_robot", 0.7f, 0.2f, 1.0f, 0.8f)
            );
        }

        if (player1_.has_odom && player1_lidar_.has_scan) { // vykreslenie lidar bodov hraca 1
            array.markers.push_back(
                createLidarHitsMarker(
                    player1_, player1_lidar_, 0,
                    "player1_lidar_hits",
                    1.0f, 0.0f, 0.0f
                )
            );
        }

        if (player2_.has_odom && player2_lidar_.has_scan) { // vykreslenie lidar bodov hraca 2
            array.markers.push_back(
                createLidarHitsMarker(
                    player2_, player2_lidar_, 0,
                    "player2_lidar_hits",
                    0.0f, 1.0f, 1.0f
                )
            );
        }

        if (has_game_state_) { // ak uz mame stav hry, mozeme kreslit herne objekty
            array.markers.push_back(createStationMarker()); // stanica

            for (size_t i = 0; i < last_game_state_.trash_id.size(); ++i) { // vsetky odpadky
                if (i < last_game_state_.trash_x.size() &&
                    i < last_game_state_.trash_y.size() &&
                    i < last_game_state_.trash_radius.size() &&
                    i < last_game_state_.trash_type.size() &&
                    i < last_game_state_.trash_collected.size()) { // kontrola velkosti poli
                    array.markers.push_back(createTrashMarker(i)); // vytvorenie markera odpadu
                }
            }

            int obstacle_id = 0; // id pre prekazky, aby kazda mala vlastny marker

            for (size_t i = 0; i < last_game_state_.circle_obstacles_x.size(); ++i) { // kruhove prekazky
                if (i < last_game_state_.circle_obstacles_y.size() &&
                    i < last_game_state_.circle_obstacles_radius.size()) {
                    array.markers.push_back(createCircleObstacleMarker(i, obstacle_id));
                    obstacle_id++;
                }
            }

            for (size_t i = 0; i < last_game_state_.rectangle_obstacles_x.size(); ++i) { // obdlznikove prekazky
                if (i < last_game_state_.rectangle_obstacles_y.size() &&
                    i < last_game_state_.rectangle_obstacles_width.size() &&
                    i < last_game_state_.rectangle_obstacles_height.size()) {
                    array.markers.push_back(createRectangleObstacleMarker(i, obstacle_id));
                    obstacle_id++;
                }
            }

            array.markers.push_back(
                createPathMarker(
                    last_game_state_.player1_path_x,
                    last_game_state_.player1_path_y,
                    0,
                    "player1_path",
                    0.0f,
                    0.3f,
                    1.0f
                )
            ); // prejdena draha hraca 1

            array.markers.push_back(
                createPathMarker(
                    last_game_state_.player2_path_x,
                    last_game_state_.player2_path_y,
                    0,
                    "player2_path",
                    0.7f,
                    0.2f,
                    1.0f
                )
            ); // prejdena draha hraca 2

            array.markers.push_back(createScoreTextMarker()); // text so skore a stavom hry
        }

        marker_pub_->publish(array); // publikovanie vsetkych markerov
    }

    RobotVisualState player1_; // ulozeny vizualny stav hraca 1
    RobotVisualState player2_; // ulozeny vizualny stav hraca 2

    LidarVisualState player1_lidar_; // ulozene lidar data hraca 1
    LidarVisualState player2_lidar_; // ulozene lidar data hraca 2

    zadanie2_interfaces::msg::GameState last_game_state_; // posledny prijaty stav hry
    bool has_game_state_ = false; // ci uz prisiel stav hry

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr player1_odom_sub_; // subscriber odometrie hraca 1
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr player2_odom_sub_; // subscriber odometrie hraca 2

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr player1_scan_sub_; // subscriber lidaru hraca 1
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr player2_scan_sub_; // subscriber lidaru hraca 2

    rclcpp::Subscription<zadanie2_interfaces::msg::GameState>::SharedPtr game_state_sub_; // subscriber stavu hry

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_; // publisher markerov do RViz
    rclcpp::TimerBase::SharedPtr timer_; // timer na periodicke publikovanie markerov
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv); // inicializacia ROS2
    rclcpp::spin(std::make_shared<VisualizationNode>()); // spustenie visualization node
    rclcpp::shutdown(); // ukoncenie ROS2
    return 0;
}