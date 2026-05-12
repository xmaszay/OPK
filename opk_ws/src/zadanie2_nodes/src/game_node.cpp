/*
- načíta parametre z YAML
- odoberá /player1/odom a /player2/odom
- posiela polohu robotov do GameLogic
- publikuje /game_state
- poskytuje /reset_game service
*/
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "zadanie2_interfaces/msg/game_state.hpp"
#include "zadanie2_interfaces/srv/reset_game.hpp"

#include "game/GameLogic.h"

class GameNode : public rclcpp::Node
{
public:
    GameNode()
        : Node("game_node") // vytvorenie ROS node s nazvom game_node
    {
        declareRequiredParameters(); // deklarujeme vsetky parametre, ktore ocakavame z yaml

        game::GameConfig config; // konfiguracna struktura pre cistu hernu logiku GameLogic

        config.map_path = getRequiredParameter<std::string>("map_path"); // cesta k png mape z yaml
        config.map_resolution = getRequiredParameter<double>("map_resolution"); // rozlisenie mapy

        config.max_capacity = getRequiredParameter<int>("max_capacity"); // maximalna kapacita robota
        config.trash_count = getRequiredParameter<int>("trash_count"); // pocet generovanych odpadkov

        config.trash_radius_min = getRequiredParameter<double>("trash_radius_min"); // minimalny polomer odpadu
        config.trash_radius_max = getRequiredParameter<double>("trash_radius_max"); // maximalny polomer odpadu
        config.trash_types = getRequiredParameter<std::vector<std::string>>("trash_types"); // typy odpadu z yaml, napr paper/plastic/glass

        config.collect_distance = getRequiredParameter<double>("collect_distance"); // vzdialenost na pozbieranie odpadu
        config.path_min_step = getRequiredParameter<double>("path_min_step"); // minimalny krok, po ktorom sa ulozi dalsi bod prejdenej drahy

        config.station_x = getRequiredParameter<double>("station_x"); // poloha stanice x
        config.station_y = getRequiredParameter<double>("station_y"); // poloha stanice y
        config.station_radius = getRequiredParameter<double>("station_radius"); // polomer stanice

        config.circle_obstacles_x =
            getRequiredParameter<std::vector<double>>("circle_obstacles_x"); // x suradnice kruhovych prekazok
        config.circle_obstacles_y =
            getRequiredParameter<std::vector<double>>("circle_obstacles_y"); // y suradnice kruhovych prekazok
        config.circle_obstacles_radius =
            getRequiredParameter<std::vector<double>>("circle_obstacles_radius"); // polomery kruhovych prekazok

        config.rectangle_obstacles_x =
            getRequiredParameter<std::vector<double>>("rectangle_obstacles_x"); // x suradnice obdlznikovych prekazok
        config.rectangle_obstacles_y =
            getRequiredParameter<std::vector<double>>("rectangle_obstacles_y"); // y suradnice obdlznikovych prekazok
        config.rectangle_obstacles_width =
            getRequiredParameter<std::vector<double>>("rectangle_obstacles_width"); // sirky obdlznikovych prekazok
        config.rectangle_obstacles_height =
            getRequiredParameter<std::vector<double>>("rectangle_obstacles_height"); // vysky obdlznikovych prekazok

        game_logic_ = std::make_unique<game::GameLogic>(config); // vytvorenie hernej logiky mimo ROS, GameNode je len wrapper

        player1_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/player1/odom", // odobera polohu prveho robota
            10,
            std::bind(&GameNode::player1OdomCallback, this, std::placeholders::_1)
        );

        player2_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/player2/odom", // odobera polohu druheho robota/bota
            10,
            std::bind(&GameNode::player2OdomCallback, this, std::placeholders::_1)
        );

        game_state_pub_ = this->create_publisher<zadanie2_interfaces::msg::GameState>(
            "game_state", // publikuje stav hry na /game_state
            10
        );

        reset_service_ = this->create_service<zadanie2_interfaces::srv::ResetGame>(
            "reset_game", // service na reset hry
            std::bind(
                &GameNode::resetGameCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // kazdych 100ms sa aktualizuje hra a publikuje stav hry
            std::bind(&GameNode::timerCallback, this)
        );

        RCLCPP_INFO(this->get_logger(), "game_node started as ROS wrapper around GameLogic.");
    }

private:
    void declareRequiredParameters() // deklaracia vsetkych parametrov, ktore musia prist z yaml
    {
        this->declare_parameter<std::string>("map_path"); // cesta k mape
        this->declare_parameter<double>("map_resolution"); // rozlisenie mapy

        this->declare_parameter<int>("max_capacity"); // maximalna kapacita robota
        this->declare_parameter<int>("trash_count"); // pocet odpadkov

        this->declare_parameter<double>("trash_radius_min"); // min polomer odpadu
        this->declare_parameter<double>("trash_radius_max"); // max polomer odpadu
        this->declare_parameter<std::vector<std::string>>("trash_types"); // druhy odpadu

        this->declare_parameter<double>("collect_distance"); // vzdialenost pre pozbieranie odpadu
        this->declare_parameter<double>("path_min_step"); // ako casto sa uklada bod drahy

        this->declare_parameter<double>("station_x"); // poloha stanice x
        this->declare_parameter<double>("station_y"); // poloha stanice y
        this->declare_parameter<double>("station_radius"); // polomer stanice

        this->declare_parameter<std::vector<double>>("circle_obstacles_x"); // kruhove prekazky - x
        this->declare_parameter<std::vector<double>>("circle_obstacles_y"); // kruhove prekazky - y
        this->declare_parameter<std::vector<double>>("circle_obstacles_radius"); // kruhove prekazky - polomer

        this->declare_parameter<std::vector<double>>("rectangle_obstacles_x"); // obdlznikove prekazky - x
        this->declare_parameter<std::vector<double>>("rectangle_obstacles_y"); // obdlznikove prekazky - y
        this->declare_parameter<std::vector<double>>("rectangle_obstacles_width"); // obdlznikove prekazky - sirka
        this->declare_parameter<std::vector<double>>("rectangle_obstacles_height"); // obdlznikove prekazky - vyska
    }

    template<typename T>
    T getRequiredParameter(const std::string& name) // funkcia na povinne citanie parametrov z yaml
    {
        rclcpp::Parameter parameter;

        if (!this->get_parameter(name, parameter) ||
            parameter.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
            throw std::runtime_error("Missing required ROS parameter: " + name); // ak parameter chyba, node skonci s chybou
        }

        return parameter.get_value<T>(); // vrati hodnotu parametra v pozadovanom type
    }

    void player1OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) // zavola sa vzdy ked pride nova odometria z /player1/odom
    {
        game_logic_->updatePlayer1Position(
            msg->pose.pose.position.x, // posleme aktualne x hraca 1 do GameLogic
            msg->pose.pose.position.y  // posleme aktualne y hraca 1 do GameLogic
        );
    }

    void player2OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) // zavola sa vzdy ked pride nova odometria z /player2/odom
    {
        game_logic_->updatePlayer2Position(
            msg->pose.pose.position.x, // posleme aktualne x hraca 2 do GameLogic
            msg->pose.pose.position.y  // posleme aktualne y hraca 2 do GameLogic
        );
    }

    void resetGameCallback(
        const std::shared_ptr<zadanie2_interfaces::srv::ResetGame::Request> request,
        std::shared_ptr<zadanie2_interfaces::srv::ResetGame::Response> response)
    {
        (void)request; // request nepouzivame, reset je bez vstupnych dat

        game_logic_->reset(); // resetuje hernu logiku - skore, kapacity, odpadky, drahy

        response->success = true; // odpoved pre service klienta
        response->message = "Game was reset successfully.";

        RCLCPP_WARN(this->get_logger(), "Game reset service was called.");
    }

    void timerCallback() // funkcia volana timerom kazdych 100ms
    {
        game_logic_->update(); // aktualizacia hry - zber odpadu, vylozenie, kontrola konca hry
        publishGameState(); // publikovanie aktualneho stavu hry
    }

    void publishGameState() // prevedie stav z GameLogic do ROS spravy GameState
    {
        game::GameSnapshot snapshot = game_logic_->getSnapshot(); // ziskame aktualny snapshot hry z cistej logiky

        zadanie2_interfaces::msg::GameState msg; // vlastna ROS sprava pre stav hry

        msg.player1_score = snapshot.player1.score; // skore hraca 1
        msg.player2_score = snapshot.player2.score; // skore hraca 2

        msg.player1_capacity = snapshot.player1.current_capacity; // aktualna kapacita hraca 1
        msg.player2_capacity = snapshot.player2.current_capacity; // aktualna kapacita hraca 2

        msg.player1_paper_count = snapshot.player1.paper_count; // pocet pozbieranych papierov hracom 1
        msg.player1_plastic_count = snapshot.player1.plastic_count; // pocet pozbieranych plastov hracom 1
        msg.player1_glass_count = snapshot.player1.glass_count; // pocet pozbieranych skiel hracom 1

        msg.player2_paper_count = snapshot.player2.paper_count; // pocet pozbieranych papierov hracom 2
        msg.player2_plastic_count = snapshot.player2.plastic_count; // pocet pozbieranych plastov hracom 2
        msg.player2_glass_count = snapshot.player2.glass_count; // pocet pozbieranych skiel hracom 2

        msg.remaining_trash = snapshot.remaining_trash; // pocet odpadkov, ktore este ostali na ploche
        msg.game_finished = snapshot.game_finished; // ci hra skoncila
        msg.status = snapshot.status; // textovy stav hry, napr RUNNING alebo vitaz

        for (const auto& item : snapshot.trash) { // naplnenie poli pre vsetky odpadky
            msg.trash_id.push_back(item.id); // id odpadu
            msg.trash_type.push_back(item.type); // typ odpadu
            msg.trash_x.push_back(item.x); // poloha odpadu x
            msg.trash_y.push_back(item.y); // poloha odpadu y
            msg.trash_radius.push_back(item.radius); // polomer odpadu
            msg.trash_collected.push_back(item.collected); // ci je odpad pozbierany
        }

        msg.station_x = snapshot.station_x; // poloha stanice x
        msg.station_y = snapshot.station_y; // poloha stanice y
        msg.station_radius = snapshot.station_radius; // polomer stanice

        msg.circle_obstacles_x = snapshot.circle_obstacles_x; // x suradnice kruhovych prekazok
        msg.circle_obstacles_y = snapshot.circle_obstacles_y; // y suradnice kruhovych prekazok
        msg.circle_obstacles_radius = snapshot.circle_obstacles_radius; // polomery kruhovych prekazok

        msg.rectangle_obstacles_x = snapshot.rectangle_obstacles_x; // x suradnice obdlznikovych prekazok
        msg.rectangle_obstacles_y = snapshot.rectangle_obstacles_y; // y suradnice obdlznikovych prekazok
        msg.rectangle_obstacles_width = snapshot.rectangle_obstacles_width; // sirky obdlznikovych prekazok
        msg.rectangle_obstacles_height = snapshot.rectangle_obstacles_height; // vysky obdlznikovych prekazok

        msg.player1_path_x = snapshot.player1.path_x; // prejdena draha hraca 1 - x body
        msg.player1_path_y = snapshot.player1.path_y; // prejdena draha hraca 1 - y body
        msg.player2_path_x = snapshot.player2.path_x; // prejdena draha hraca 2 - x body
        msg.player2_path_y = snapshot.player2.path_y; // prejdena draha hraca 2 - y body

        game_state_pub_->publish(msg); // publikovanie stavu hry na /game_state
    }

    std::unique_ptr<game::GameLogic> game_logic_; // cista herna logika bez ROS

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr player1_odom_sub_; // subscriber na odometriu hraca 1
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr player2_odom_sub_; // subscriber na odometriu hraca 2

    rclcpp::Publisher<zadanie2_interfaces::msg::GameState>::SharedPtr game_state_pub_; // publisher stavu hry
    rclcpp::Service<zadanie2_interfaces::srv::ResetGame>::SharedPtr reset_service_; // service na reset hry

    rclcpp::TimerBase::SharedPtr timer_; // timer na periodicku aktualizaciu hry
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv); // inicializacia ROS2
    rclcpp::spin(std::make_shared<GameNode>()); // spustenie node a cakanie na callbacky
    rclcpp::shutdown(); // ukoncenie ROS2
    return 0;
}