#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "environment/Environment.h"

struct Trash
{
    int id;
    std::string type;
    double x;
    double y;
    double radius;
    bool collected;
};

struct PlayerState
{
    double x = 0.0;
    double y = 0.0;
    bool has_odom = false;

    int current_capacity = 0;
    int max_capacity = 3;

    int paper_count = 0;
    int plastic_count = 0;
    int glass_count = 0;

    int score = 0;
};

class GameNode : public rclcpp::Node
{
public:
    GameNode()
        : Node("game_node"),
          rng_(std::random_device{}())
    {
        this->declare_parameter<std::string>(
            "map_path",
            "/home/peter/Desktop/OPK/opk_ws/src/zadanie1/resources/opk-map.png"
        );
        this->declare_parameter<double>("map_resolution", 0.02);

        this->declare_parameter<int>("max_capacity", 3);
        this->declare_parameter<int>("trash_count", 12);

        this->declare_parameter<double>("trash_radius", 0.25);
        this->declare_parameter<double>("collect_distance", 0.7);

        this->declare_parameter<double>("station_x", 21.0);
        this->declare_parameter<double>("station_y", 7.5);
        this->declare_parameter<double>("station_radius", 0.8);

        environment::Config env_config;
        env_config.map_filename = this->get_parameter("map_path").as_string();
        env_config.resolution = this->get_parameter("map_resolution").as_double();

        env_ = std::make_shared<environment::Environment>(env_config);

        player1_.max_capacity = this->get_parameter("max_capacity").as_int();
        player2_.max_capacity = this->get_parameter("max_capacity").as_int();

        trash_radius_ = this->get_parameter("trash_radius").as_double();
        collect_distance_ = this->get_parameter("collect_distance").as_double();

        station_x_ = this->get_parameter("station_x").as_double();
        station_y_ = this->get_parameter("station_y").as_double();
        station_radius_ = this->get_parameter("station_radius").as_double();

        player1_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/player1/odom",
            10,
            std::bind(&GameNode::player1OdomCallback, this, std::placeholders::_1)
        );

        player2_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/player2/odom",
            10,
            std::bind(&GameNode::player2OdomCallback, this, std::placeholders::_1)
        );

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "game_markers",
            10
        );

        int trash_count = this->get_parameter("trash_count").as_int();
        generateTrash(trash_count);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&GameNode::updateGame, this)
        );

        RCLCPP_INFO(
            this->get_logger(),
            "game_node started. Generated %zu trash objects only on free map cells.",
            trash_.size()
        );
    }

private:
    void generateTrash(int count)
    {
        trash_.clear();

        std::uniform_real_distribution<double> x_dist(0.0, env_->getWidth());
        std::uniform_real_distribution<double> y_dist(0.0, env_->getHeight());

        std::vector<std::string> types = {"paper", "plastic", "glass"};

        int id = 0;
        int attempts = 0;
        const int max_attempts = count * 1000;

        while (id < count && attempts < max_attempts) {
            attempts++;

            double x = x_dist(rng_);
            double y = y_dist(rng_);

            if (!isValidSpawnPosition(x, y)) {
                continue;
            }

            std::string type = types[id % static_cast<int>(types.size())];

            trash_.push_back({
                id,
                type,
                x,
                y,
                trash_radius_,
                false
            });

            id++;
        }

        if (id < count) {
            RCLCPP_WARN(
                this->get_logger(),
                "Could only generate %d/%d trash objects.",
                id,
                count
            );
        }
    }

    bool isValidSpawnPosition(double x, double y) const
    {
        if (env_->isOccupied(x, y)) {
            return false;
        }

        if (distance(x, y, station_x_, station_y_) < station_radius_ + 1.0) {
            return false;
        }

        for (const auto& item : trash_) {
            if (distance(x, y, item.x, item.y) < 1.0) {
                return false;
            }
        }

        const double safety = 0.35;

        if (env_->isOccupied(x + safety, y)) return false;
        if (env_->isOccupied(x - safety, y)) return false;
        if (env_->isOccupied(x, y + safety)) return false;
        if (env_->isOccupied(x, y - safety)) return false;

        return true;
    }

    void player1OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        player1_.x = msg->pose.pose.position.x;
        player1_.y = msg->pose.pose.position.y;
        player1_.has_odom = true;
    }

    void player2OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        player2_.x = msg->pose.pose.position.x;
        player2_.y = msg->pose.pose.position.y;
        player2_.has_odom = true;
    }

    double distance(double x1, double y1, double x2, double y2) const
    {
        double dx = x1 - x2;
        double dy = y1 - y2;
        return std::sqrt(dx * dx + dy * dy);
    }

    void updateGame()
    {
        if (!game_finished_) {
            if (player1_.has_odom) {
                handlePlayer(player1_, "Player 1");
            }

            if (player2_.has_odom) {
                handlePlayer(player2_, "Player 2");
            }

            checkGameFinished();
        }

        publishMarkers();
    }

    void handlePlayer(PlayerState& player, const std::string& player_name)
    {
        collectTrash(player, player_name);
        unloadAtStation(player, player_name);
    }

    void collectTrash(PlayerState& player, const std::string& player_name)
    {
        if (player.current_capacity >= player.max_capacity) {
            return;
        }

        for (auto& item : trash_) {
            if (item.collected) {
                continue;
            }

            double d = distance(player.x, player.y, item.x, item.y);

            if (d <= collect_distance_ + item.radius) {
                item.collected = true;
                player.current_capacity++;

                if (item.type == "paper") {
                    player.paper_count++;
                } else if (item.type == "plastic") {
                    player.plastic_count++;
                } else if (item.type == "glass") {
                    player.glass_count++;
                }

                RCLCPP_INFO(
                    this->get_logger(),
                    "%s collected %s. Capacity: %d/%d",
                    player_name.c_str(),
                    item.type.c_str(),
                    player.current_capacity,
                    player.max_capacity
                );

                return;
            }
        }
    }

    void unloadAtStation(PlayerState& player, const std::string& player_name)
    {
        if (player.current_capacity <= 0) {
            return;
        }

        double d = distance(player.x, player.y, station_x_, station_y_);

        if (d <= station_radius_) {
            player.score += player.current_capacity;

            RCLCPP_INFO(
                this->get_logger(),
                "%s unloaded %d trash. Score: %d",
                player_name.c_str(),
                player.current_capacity,
                player.score
            );

            player.current_capacity = 0;
        }
    }

    void checkGameFinished()
    {
        bool all_collected = true;

        for (const auto& item : trash_) {
            if (!item.collected) {
                all_collected = false;
                break;
            }
        }

        if (!all_collected) {
            return;
        }

        if (player1_.current_capacity > 0 || player2_.current_capacity > 0) {
            return;
        }

        game_finished_ = true;

        std::string winner;

        if (player1_.score > player2_.score) {
            winner = "PLAYER 1 WINS!";
        } else if (player2_.score > player1_.score) {
            winner = "PLAYER 2 WINS!";
        } else {
            winner = "DRAW!";
        }

        RCLCPP_WARN(
            this->get_logger(),
            "GAME OVER. %s Final score: P1=%d, P2=%d",
            winner.c_str(),
            player1_.score,
            player2_.score
        );
    }

    visualization_msgs::msg::Marker createTrashMarker(const Trash& item)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = "map";

        marker.ns = "trash";
        marker.id = item.id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;

        marker.action = item.collected
            ? visualization_msgs::msg::Marker::DELETE
            : visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = item.x;
        marker.pose.position.y = item.y;
        marker.pose.position.z = item.radius;

        marker.pose.orientation.w = 1.0;

        marker.scale.x = item.radius * 2.0;
        marker.scale.y = item.radius * 2.0;
        marker.scale.z = item.radius * 2.0;

        if (item.type == "paper") {
            marker.color.r = 1.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.2f;
        } else if (item.type == "plastic") {
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

        marker.pose.position.x = station_x_;
        marker.pose.position.y = station_y_;
        marker.pose.position.z = 0.05;

        marker.pose.orientation.w = 1.0;

        marker.scale.x = station_radius_ * 2.0;
        marker.scale.y = station_radius_ * 2.0;
        marker.scale.z = 0.1;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.8f;

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

        std::string status = "RUNNING";

        if (game_finished_) {
            if (player1_.score > player2_.score) {
                status = "PLAYER 1 WINS!";
            } else if (player2_.score > player1_.score) {
                status = "PLAYER 2 WINS!";
            } else {
                status = "DRAW!";
            }
        }

        marker.text =
            "DUEL MODE - " + status +
            "\nP1 score: " + std::to_string(player1_.score) +
            " | cap: " + std::to_string(player1_.current_capacity) +
            "/" + std::to_string(player1_.max_capacity) +
            "\nP2 score: " + std::to_string(player2_.score) +
            " | cap: " + std::to_string(player2_.current_capacity) +
            "/" + std::to_string(player2_.max_capacity);

        return marker;
    }

    void publishMarkers()
    {
        visualization_msgs::msg::MarkerArray array;

        array.markers.push_back(createStationMarker());

        for (const auto& item : trash_) {
            array.markers.push_back(createTrashMarker(item));
        }

        array.markers.push_back(createScoreTextMarker());

        marker_pub_->publish(array);
    }

    std::shared_ptr<environment::Environment> env_;

    PlayerState player1_;
    PlayerState player2_;

    std::vector<Trash> trash_;

    std::mt19937 rng_;

    bool game_finished_ = false;

    double trash_radius_;
    double collect_distance_;

    double station_x_;
    double station_y_;
    double station_radius_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr player1_odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr player2_odom_sub_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GameNode>());
    rclcpp::shutdown();
    return 0;
}