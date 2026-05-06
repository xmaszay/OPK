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

struct RobotVisualState
{
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    bool has_odom = false;
};

struct LidarVisualState
{
    std::vector<float> ranges;
    double angle_min = 0.0;
    double angle_increment = 0.0;
    double range_min = 0.0;
    double range_max = 0.0;
    bool has_scan = false;
};

class VisualizationNode : public rclcpp::Node
{
public:
    VisualizationNode()
        : Node("visualization_node")
    {
        player1_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/player1/odom",
            10,
            std::bind(&VisualizationNode::player1OdomCallback, this, std::placeholders::_1)
        );

        player2_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/player2/odom",
            10,
            std::bind(&VisualizationNode::player2OdomCallback, this, std::placeholders::_1)
        );

        player1_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/player1/scan",
            10,
            std::bind(&VisualizationNode::player1ScanCallback, this, std::placeholders::_1)
        );

        player2_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/player2/scan",
            10,
            std::bind(&VisualizationNode::player2ScanCallback, this, std::placeholders::_1)
        );

        game_state_sub_ = this->create_subscription<zadanie2_interfaces::msg::GameState>(
            "/game_state",
            10,
            std::bind(&VisualizationNode::gameStateCallback, this, std::placeholders::_1)
        );

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "visualization_markers",
            10
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&VisualizationNode::publishMarkers, this)
        );

        RCLCPP_INFO(this->get_logger(), "visualization_node started");
    }

private:
    void player1OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        updateRobotState(player1_, msg);
    }

    void player2OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        updateRobotState(player2_, msg);
    }

    void player1ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        updateLidarState(player1_lidar_, msg);
    }

    void player2ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        updateLidarState(player2_lidar_, msg);
    }

    void updateRobotState(
        RobotVisualState& state,
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        state.x = msg->pose.pose.position.x;
        state.y = msg->pose.pose.position.y;

        const auto& q = msg->pose.pose.orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        state.theta = std::atan2(siny_cosp, cosy_cosp);

        state.has_odom = true;
    }

    void updateLidarState(
        LidarVisualState& state,
        const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        state.ranges = msg->ranges;
        state.angle_min = msg->angle_min;
        state.angle_increment = msg->angle_increment;
        state.range_min = msg->range_min;
        state.range_max = msg->range_max;
        state.has_scan = true;
    }

    void gameStateCallback(const zadanie2_interfaces::msg::GameState::SharedPtr msg)
    {
        last_game_state_ = *msg;
        has_game_state_ = true;
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
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = "map";

        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = robot.x;
        marker.pose.position.y = robot.y;
        marker.pose.position.z = 0.10;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.7;
        marker.scale.y = 0.7;
        marker.scale.z = 0.2;

        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = a;

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
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = "map";

        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.05;
        marker.scale.y = 0.05;

        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0f;

        if (!robot.has_odom || !lidar.has_scan) {
            return marker;
        }

        for (size_t i = 0; i < lidar.ranges.size(); ++i) {
            double range = static_cast<double>(lidar.ranges[i]);

            if (!std::isfinite(range)) {
                continue;
            }

            if (range < lidar.range_min || range > lidar.range_max) {
                continue;
            }

            double local_angle =
                lidar.angle_min + static_cast<double>(i) * lidar.angle_increment;
            double world_angle = robot.theta + local_angle;

            geometry_msgs::msg::Point p;
            p.x = robot.x + range * std::cos(world_angle);
            p.y = robot.y + range * std::sin(world_angle);
            p.z = 0.03;

            marker.points.push_back(p);
        }

        return marker;
    }

    visualization_msgs::msg::Marker createTrashMarker(size_t i)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = "map";

        marker.ns = "trash";
        marker.id = last_game_state_.trash_id[i];
        marker.type = visualization_msgs::msg::Marker::SPHERE;

        marker.action = last_game_state_.trash_collected[i]
            ? visualization_msgs::msg::Marker::DELETE
            : visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = last_game_state_.trash_x[i];
        marker.pose.position.y = last_game_state_.trash_y[i];
        marker.pose.position.z = last_game_state_.trash_radius[i];
        marker.pose.orientation.w = 1.0;

        marker.scale.x = last_game_state_.trash_radius[i] * 2.0;
        marker.scale.y = last_game_state_.trash_radius[i] * 2.0;
        marker.scale.z = last_game_state_.trash_radius[i] * 2.0;

        const std::string& type = last_game_state_.trash_type[i];

        if (type == "paper") {
            marker.color.r = 1.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.2f;
        } else if (type == "plastic") {
            marker.color.r = 1.0f;
            marker.color.g = 0.7f;
            marker.color.b = 0.0f;
        } else {
            marker.color.r = 0.4f;
            marker.color.g = 1.0f;
            marker.color.b = 0.4f;
        }

        marker.color.a = 1.0f;
        return marker;
    }

    visualization_msgs::msg::Marker createStationMarker()
    {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = "map";

        marker.ns = "station";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = last_game_state_.station_x;
        marker.pose.position.y = last_game_state_.station_y;
        marker.pose.position.z = 0.05;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = last_game_state_.station_radius * 2.0;
        marker.scale.y = last_game_state_.station_radius * 2.0;
        marker.scale.z = 0.1;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.8f;

        return marker;
    }

    visualization_msgs::msg::Marker createCircleObstacleMarker(size_t i, int id)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = "map";

        marker.ns = "obstacles";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = last_game_state_.circle_obstacles_x[i];
        marker.pose.position.y = last_game_state_.circle_obstacles_y[i];
        marker.pose.position.z = 0.08;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = last_game_state_.circle_obstacles_radius[i] * 2.0;
        marker.scale.y = last_game_state_.circle_obstacles_radius[i] * 2.0;
        marker.scale.z = 0.15;

        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.7f;

        return marker;
    }

    visualization_msgs::msg::Marker createRectangleObstacleMarker(size_t i, int id)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = "map";

        marker.ns = "obstacles";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = last_game_state_.rectangle_obstacles_x[i];
        marker.pose.position.y = last_game_state_.rectangle_obstacles_y[i];
        marker.pose.position.z = 0.08;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = last_game_state_.rectangle_obstacles_width[i];
        marker.scale.y = last_game_state_.rectangle_obstacles_height[i];
        marker.scale.z = 0.15;

        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.7f;

        return marker;
    }

    visualization_msgs::msg::Marker createScoreTextMarker()
    {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = "map";

        marker.ns = "score";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = 2.0;
        marker.pose.position.y = 1.0;
        marker.pose.position.z = 1.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.z = 0.45;

        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0f;

        marker.text =
            "DUEL MODE - " + last_game_state_.status +
            "\nP1 score: " + std::to_string(last_game_state_.player1_score) +
            " | cap: " + std::to_string(last_game_state_.player1_capacity) +
            "\nP2 score: " + std::to_string(last_game_state_.player2_score) +
            " | cap: " + std::to_string(last_game_state_.player2_capacity) +
            "\nRemaining trash: " + std::to_string(last_game_state_.remaining_trash);

        return marker;
    }

    void publishMarkers()
    {
        visualization_msgs::msg::MarkerArray array;

        if (player1_.has_odom) {
            array.markers.push_back(
                createRobotMarker(player1_, 0, "player1_robot", 0.0f, 0.3f, 1.0f, 1.0f)
            );
        }

        if (player2_.has_odom) {
            array.markers.push_back(
                createRobotMarker(player2_, 0, "player2_robot", 0.7f, 0.2f, 1.0f, 0.8f)
            );
        }

        if (player1_.has_odom && player1_lidar_.has_scan) {
            array.markers.push_back(
                createLidarHitsMarker(
                    player1_, player1_lidar_, 0,
                    "player1_lidar_hits",
                    1.0f, 0.0f, 0.0f
                )
            );
        }

        if (player2_.has_odom && player2_lidar_.has_scan) {
            array.markers.push_back(
                createLidarHitsMarker(
                    player2_, player2_lidar_, 0,
                    "player2_lidar_hits",
                    0.0f, 1.0f, 1.0f
                )
            );
        }

        if (has_game_state_) {
            array.markers.push_back(createStationMarker());

            for (size_t i = 0; i < last_game_state_.trash_id.size(); ++i) {
                if (i < last_game_state_.trash_x.size() &&
                    i < last_game_state_.trash_y.size() &&
                    i < last_game_state_.trash_radius.size() &&
                    i < last_game_state_.trash_type.size() &&
                    i < last_game_state_.trash_collected.size()) {
                    array.markers.push_back(createTrashMarker(i));
                }
            }

            int obstacle_id = 0;

            for (size_t i = 0; i < last_game_state_.circle_obstacles_x.size(); ++i) {
                if (i < last_game_state_.circle_obstacles_y.size() &&
                    i < last_game_state_.circle_obstacles_radius.size()) {
                    array.markers.push_back(createCircleObstacleMarker(i, obstacle_id));
                    obstacle_id++;
                }
            }

            for (size_t i = 0; i < last_game_state_.rectangle_obstacles_x.size(); ++i) {
                if (i < last_game_state_.rectangle_obstacles_y.size() &&
                    i < last_game_state_.rectangle_obstacles_width.size() &&
                    i < last_game_state_.rectangle_obstacles_height.size()) {
                    array.markers.push_back(createRectangleObstacleMarker(i, obstacle_id));
                    obstacle_id++;
                }
            }

            array.markers.push_back(createScoreTextMarker());
        }

        marker_pub_->publish(array);
    }

    RobotVisualState player1_;
    RobotVisualState player2_;

    LidarVisualState player1_lidar_;
    LidarVisualState player2_lidar_;

    zadanie2_interfaces::msg::GameState last_game_state_;
    bool has_game_state_ = false;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr player1_odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr player2_odom_sub_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr player1_scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr player2_scan_sub_;

    rclcpp::Subscription<zadanie2_interfaces::msg::GameState>::SharedPtr game_state_sub_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualizationNode>());
    rclcpp::shutdown();
    return 0;
}