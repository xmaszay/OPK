#pragma once

#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "environment/Environment.h"

namespace game {

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

    std::vector<double> path_x;
    std::vector<double> path_y;
};

struct GameSnapshot
{
    PlayerState player1;
    PlayerState player2;

    std::vector<Trash> trash;

    int remaining_trash = 0;

    bool game_finished = false;
    std::string status = "RUNNING";

    double station_x = 0.0;
    double station_y = 0.0;
    double station_radius = 0.0;

    std::vector<double> circle_obstacles_x;
    std::vector<double> circle_obstacles_y;
    std::vector<double> circle_obstacles_radius;

    std::vector<double> rectangle_obstacles_x;
    std::vector<double> rectangle_obstacles_y;
    std::vector<double> rectangle_obstacles_width;
    std::vector<double> rectangle_obstacles_height;
};

struct GameConfig
{
    std::string map_path;
    double map_resolution = 0.0;

    int max_capacity = 0;
    int trash_count = 0;

    double trash_radius_min = 0.0;
    double trash_radius_max = 0.0;
    std::vector<std::string> trash_types;

    double collect_distance = 0.0;
    double path_min_step = 0.05;

    double station_x = 0.0;
    double station_y = 0.0;
    double station_radius = 0.0;

    std::vector<double> circle_obstacles_x;
    std::vector<double> circle_obstacles_y;
    std::vector<double> circle_obstacles_radius;

    std::vector<double> rectangle_obstacles_x;
    std::vector<double> rectangle_obstacles_y;
    std::vector<double> rectangle_obstacles_width;
    std::vector<double> rectangle_obstacles_height;
};

class TrashFactory
{
public:
    static Trash createTrash(
        int id,
        const std::string& type,
        double x,
        double y,
        double radius);
};

class GameObject
{
public:
    GameObject(double x, double y, double radius);
    virtual ~GameObject() = default;

    double getX() const;
    double getY() const;
    double getRadius() const;

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
    StationObject(double x, double y, double radius);

    std::string getObjectType() const override;
    bool contains(double x, double y) const override;
};

class CircleObstacleObject : public GameObject
{
public:
    CircleObstacleObject(double x, double y, double radius);

    std::string getObjectType() const override;
    bool contains(double x, double y) const override;
};

class RectangleObstacleObject : public GameObject
{
public:
    RectangleObstacleObject(double x, double y, double width, double height);

    std::string getObjectType() const override;
    bool contains(double x, double y) const override;

    double getWidth() const;
    double getHeight() const;

private:
    double width_;
    double height_;
};

class GameLogic
{
public:
    explicit GameLogic(const GameConfig& config);

    void reset();

    void updatePlayer1Position(double x, double y);
    void updatePlayer2Position(double x, double y);

    void update();

    GameSnapshot getSnapshot() const;

private:
    void validateConfig() const;
    void loadObstacles();
    void generateTrash();

    void addPathPoint(PlayerState& player, double x, double y);

    void handlePlayer(PlayerState& player);
    void collectTrash(PlayerState& player);
    void unloadAtStation(PlayerState& player);

    void checkGameFinished();

    bool isValidSpawnPosition(double x, double y) const;
    bool isInsideAnyObstacle(double x, double y) const;

    double distance(double x1, double y1, double x2, double y2) const;

    int getRemainingTrashCount() const;
    std::string getGameStatus() const;

    GameConfig config_;

    std::shared_ptr<environment::Environment> env_;

    PlayerState player1_;
    PlayerState player2_;

    std::vector<Trash> trash_;

    std::unique_ptr<StationObject> station_;
    std::vector<std::unique_ptr<GameObject>> obstacles_;

    std::mt19937 rng_;

    bool game_finished_ = false;
};

} // namespace game
