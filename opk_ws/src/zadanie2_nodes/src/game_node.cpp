#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "zadanie2_interfaces/msg/game_state.hpp"
#include "zadanie2_interfaces/srv/reset_game.hpp"

#include "environment/Environment.h"

class GameConfigException : public std::runtime_error
{
public:
    explicit GameConfigException(const std::string& message)
        : std::runtime_error(message)
    {
    }
};

struct Trash
{
    int id;
    std::string type;
    double x;
    double y;
    double radius;
    bool collected;
};

class TrashFactory
{
public:
    static Trash createTrash(
        int id,
        const std::string& type,
        double x,
        double y,
        double radius)
    {
        if (type != "paper" && type != "plastic" && type != "glass") {
            throw GameConfigException("Unknown trash type: " + type);
        }

        return Trash{
            id,
            type,
            x,
            y,
            radius,
            false
        };
    }
};

class GameObject
{
public:
    GameObject(double x, double y, double radius)
        : x_(x), y_(y), radius_(radius)
    {
    }

    virtual ~GameObject() = default;

    double getX() const { return x_; }
    double getY() const { return y_; }
    double getRadius() const { return radius_; }

    virtual std::string getObjectType() const = 0;
    virtual bool contains(double x, double y) const = 0;

protected:
    double x_;
    double y_;
    double radius_;
};

class StationObject : public GameObject
{
public:
    StationObject(double x, double y, double radius)
        : GameObject(x, y, radius)
    {
    }

    std::string getObjectType() const override
    {
        return "station";
    }

    bool contains(double x, double y) const override
    {
        double dx = x - x_;
        double dy = y - y_;
        return std::sqrt(dx * dx + dy * dy) <= radius_;
    }
};

class CircleObstacleObject : public GameObject
{
public:
    CircleObstacleObject(double x, double y, double radius)
        : GameObject(x, y, radius)
    {
    }

    std::string getObjectType() const override
    {
        return "circle_obstacle";
    }

    bool contains(double x, double y) const override
    {
        double dx = x - x_;
        double dy = y - y_;
        return std::sqrt(dx * dx + dy * dy) <= radius_;
    }
};

class RectangleObstacleObject : public GameObject
{
public:
    RectangleObstacleObject(double x, double y, double width, double height)
        : GameObject(x, y, 0.0), width_(width), height_(height)
    {
    }

    std::string getObjectType() const override
    {
        return "rectangle_obstacle";
    }

    bool contains(double x, double y) const override
    {
        return x >= x_ - width_ * 0.5 &&
               x <= x_ + width_ * 0.5 &&
               y >= y_ - height_ * 0.5 &&
               y <= y_ + height_ * 0.5;
    }

    double getWidth() const { return width_; }
    double getHeight() const { return height_; }

private:
    double width_;
    double height_;
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

        this->declare_parameter<std::vector<double>>("circle_obstacles_x", std::vector<double>{});
        this->declare_parameter<std::vector<double>>("circle_obstacles_y", std::vector<double>{});
        this->declare_parameter<std::vector<double>>("circle_obstacles_radius", std::vector<double>{});

        this->declare_parameter<std::vector<double>>("rectangle_obstacles_x", std::vector<double>{});
        this->declare_parameter<std::vector<double>>("rectangle_obstacles_y", std::vector<double>{});
        this->declare_parameter<std::vector<double>>("rectangle_obstacles_width", std::vector<double>{});
        this->declare_parameter<std::vector<double>>("rectangle_obstacles_height", std::vector<double>{});

        environment::Config env_config;
        env_config.map_filename = this->get_parameter("map_path").as_string();
        env_config.resolution = this->get_parameter("map_resolution").as_double();

        validateConfig(env_config);

        env_ = std::make_shared<environment::Environment>(env_config);

        player1_.max_capacity = this->get_parameter("max_capacity").as_int();
        player2_.max_capacity = this->get_parameter("max_capacity").as_int();

        trash_radius_ = this->get_parameter("trash_radius").as_double();
        collect_distance_ = this->get_parameter("collect_distance").as_double();

        station_x_ = this->get_parameter("station_x").as_double();
        station_y_ = this->get_parameter("station_y").as_double();
        station_radius_ = this->get_parameter("station_radius").as_double();

        station_ = std::make_unique<StationObject>(station_x_, station_y_, station_radius_);

        loadObstaclesFromParameters();

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

        game_state_pub_ = this->create_publisher<zadanie2_interfaces::msg::GameState>(
            "game_state",
            10
        );

        reset_service_ = this->create_service<zadanie2_interfaces::srv::ResetGame>(
            "reset_game",
            std::bind(
                &GameNode::resetGameCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );

        int trash_count = this->get_parameter("trash_count").as_int();
        generateTrash(trash_count);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&GameNode::updateGame, this)
        );

        RCLCPP_INFO(
            this->get_logger(),
            "game_node started. Generated %zu trash objects.",
            trash_.size()
        );
    }

private:
    void validateConfig(const environment::Config& env_config)
    {
        if (env_config.resolution <= 0.0) {
            throw GameConfigException("map_resolution must be positive");
        }

        if (this->get_parameter("max_capacity").as_int() <= 0) {
            throw GameConfigException("max_capacity must be positive");
        }

        if (this->get_parameter("trash_count").as_int() <= 0) {
            throw GameConfigException("trash_count must be positive");
        }

        if (this->get_parameter("trash_radius").as_double() <= 0.0) {
            throw GameConfigException("trash_radius must be positive");
        }

        if (this->get_parameter("station_radius").as_double() <= 0.0) {
            throw GameConfigException("station_radius must be positive");
        }
    }

    void loadObstaclesFromParameters()
    {
        auto circle_x = this->get_parameter("circle_obstacles_x").as_double_array();
        auto circle_y = this->get_parameter("circle_obstacles_y").as_double_array();
        auto circle_r = this->get_parameter("circle_obstacles_radius").as_double_array();

        if (circle_x.size() != circle_y.size() || circle_x.size() != circle_r.size()) {
            throw GameConfigException("Circle obstacle arrays must have same size");
        }

        for (size_t i = 0; i < circle_x.size(); ++i) {
            obstacles_.push_back(
                std::make_unique<CircleObstacleObject>(circle_x[i], circle_y[i], circle_r[i])
            );

            circle_obstacles_x_.push_back(circle_x[i]);
            circle_obstacles_y_.push_back(circle_y[i]);
            circle_obstacles_radius_.push_back(circle_r[i]);
        }

        auto rect_x = this->get_parameter("rectangle_obstacles_x").as_double_array();
        auto rect_y = this->get_parameter("rectangle_obstacles_y").as_double_array();
        auto rect_w = this->get_parameter("rectangle_obstacles_width").as_double_array();
        auto rect_h = this->get_parameter("rectangle_obstacles_height").as_double_array();

        if (rect_x.size() != rect_y.size() ||
            rect_x.size() != rect_w.size() ||
            rect_x.size() != rect_h.size()) {
            throw GameConfigException("Rectangle obstacle arrays must have same size");
        }

        for (size_t i = 0; i < rect_x.size(); ++i) {
            obstacles_.push_back(
                std::make_unique<RectangleObstacleObject>(rect_x[i], rect_y[i], rect_w[i], rect_h[i])
            );

            rectangle_obstacles_x_.push_back(rect_x[i]);
            rectangle_obstacles_y_.push_back(rect_y[i]);
            rectangle_obstacles_width_.push_back(rect_w[i]);
            rectangle_obstacles_height_.push_back(rect_h[i]);
        }
    }

    void resetGameCallback(
        const std::shared_ptr<zadanie2_interfaces::srv::ResetGame::Request> request,
        std::shared_ptr<zadanie2_interfaces::srv::ResetGame::Response> response)
    {
        (void)request;

        player1_ = PlayerState{};
        player2_ = PlayerState{};

        player1_.max_capacity = this->get_parameter("max_capacity").as_int();
        player2_.max_capacity = this->get_parameter("max_capacity").as_int();

        game_finished_ = false;

        int trash_count = this->get_parameter("trash_count").as_int();
        generateTrash(trash_count);

        response->success = true;
        response->message = "Game was reset successfully.";

        RCLCPP_WARN(this->get_logger(), "Game reset service was called.");
    }

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

            trash_.push_back(
                TrashFactory::createTrash(id, type, x, y, trash_radius_)
            );

            id++;
        }
    }

    bool isInsideAnyObstacle(double x, double y) const
    {
        for (const auto& obstacle : obstacles_) {
            if (obstacle->contains(x, y)) {
                return true;
            }
        }

        return false;
    }

    bool isValidSpawnPosition(double x, double y) const
    {
        if (env_->isOccupied(x, y)) {
            return false;
        }

        if (isInsideAnyObstacle(x, y)) {
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

        if (isInsideAnyObstacle(x + safety, y)) return false;
        if (isInsideAnyObstacle(x - safety, y)) return false;
        if (isInsideAnyObstacle(x, y + safety)) return false;
        if (isInsideAnyObstacle(x, y - safety)) return false;

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

        publishGameState();
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

    int getRemainingTrashCount() const
    {
        int remaining = 0;

        for (const auto& item : trash_) {
            if (!item.collected) {
                remaining++;
            }
        }

        return remaining;
    }

    std::string getGameStatus() const
    {
        if (!game_finished_) {
            return "RUNNING";
        }

        if (player1_.score > player2_.score) {
            return "PLAYER 1 WINS!";
        }

        if (player2_.score > player1_.score) {
            return "PLAYER 2 WINS!";
        }

        return "DRAW!";
    }

    void checkGameFinished()
    {
        if (getRemainingTrashCount() > 0) {
            return;
        }

        if (player1_.current_capacity > 0 || player2_.current_capacity > 0) {
            return;
        }

        game_finished_ = true;

        RCLCPP_WARN(
            this->get_logger(),
            "GAME OVER. %s Final score: P1=%d, P2=%d",
            getGameStatus().c_str(),
            player1_.score,
            player2_.score
        );
    }

    void publishGameState()
    {
        zadanie2_interfaces::msg::GameState msg;

        msg.player1_score = player1_.score;
        msg.player2_score = player2_.score;

        msg.player1_capacity = player1_.current_capacity;
        msg.player2_capacity = player2_.current_capacity;

        msg.player1_paper_count = player1_.paper_count;
        msg.player1_plastic_count = player1_.plastic_count;
        msg.player1_glass_count = player1_.glass_count;

        msg.player2_paper_count = player2_.paper_count;
        msg.player2_plastic_count = player2_.plastic_count;
        msg.player2_glass_count = player2_.glass_count;

        msg.remaining_trash = getRemainingTrashCount();
        msg.game_finished = game_finished_;
        msg.status = getGameStatus();

        for (const auto& item : trash_) {
            msg.trash_id.push_back(item.id);
            msg.trash_type.push_back(item.type);
            msg.trash_x.push_back(item.x);
            msg.trash_y.push_back(item.y);
            msg.trash_radius.push_back(item.radius);
            msg.trash_collected.push_back(item.collected);
        }

        msg.station_x = station_->getX();
        msg.station_y = station_->getY();
        msg.station_radius = station_->getRadius();

        msg.circle_obstacles_x = circle_obstacles_x_;
        msg.circle_obstacles_y = circle_obstacles_y_;
        msg.circle_obstacles_radius = circle_obstacles_radius_;

        msg.rectangle_obstacles_x = rectangle_obstacles_x_;
        msg.rectangle_obstacles_y = rectangle_obstacles_y_;
        msg.rectangle_obstacles_width = rectangle_obstacles_width_;
        msg.rectangle_obstacles_height = rectangle_obstacles_height_;

        game_state_pub_->publish(msg);
    }

    std::shared_ptr<environment::Environment> env_;

    PlayerState player1_;
    PlayerState player2_;

    std::vector<Trash> trash_;

    std::unique_ptr<StationObject> station_;
    std::vector<std::unique_ptr<GameObject>> obstacles_;

    std::vector<double> circle_obstacles_x_;
    std::vector<double> circle_obstacles_y_;
    std::vector<double> circle_obstacles_radius_;

    std::vector<double> rectangle_obstacles_x_;
    std::vector<double> rectangle_obstacles_y_;
    std::vector<double> rectangle_obstacles_width_;
    std::vector<double> rectangle_obstacles_height_;

    std::mt19937 rng_;

    bool game_finished_ = false;

    double trash_radius_;
    double collect_distance_;

    double station_x_;
    double station_y_;
    double station_radius_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr player1_odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr player2_odom_sub_;

    rclcpp::Publisher<zadanie2_interfaces::msg::GameState>::SharedPtr game_state_pub_;
    rclcpp::Service<zadanie2_interfaces::srv::ResetGame>::SharedPtr reset_service_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GameNode>());
    rclcpp::shutdown();
    return 0;
}