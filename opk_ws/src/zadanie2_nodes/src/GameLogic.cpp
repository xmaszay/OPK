/*
- generuje odpadky
- kontroluje zber
- kontroluje vyloženie v stanici
- počíta skóre
- sleduje kapacitu
- sleduje prejdenú dráhu
- určuje koniec hry a víťaza
*/

#include "game/GameLogic.h"

#include <cmath>

namespace game {

Trash TrashFactory::createTrash(
    int id,
    const std::string& type,
    double x,
    double y,
    double radius)
{
    if (type.empty()) { // kontrola ci typ odpadu nie je prazdny
        throw GameConfigException("Trash type cannot be empty.");
    }

    return Trash{ // vytvorenie odpadu cez factory pattern
        id,
        type,
        x,
        y,
        radius,
        false // na zaciatku odpad este nie je pozbierany
    };
}

GameObject::GameObject(double x, double y, double radius)
    : x_(x), y_(y), radius_(radius) // zakladny konstruktor pre objekty prostredia
{
}

double GameObject::getX() const
{
    return x_; // vrati x suradnicu objektu
}

double GameObject::getY() const
{
    return y_; // vrati y suradnicu objektu
}

double GameObject::getRadius() const
{
    return radius_; // vrati polomer objektu
}

StationObject::StationObject(double x, double y, double radius)
    : GameObject(x, y, radius) // stanica dedi zo zakladnej triedy GameObject
{
}

std::string StationObject::getObjectType() const
{
    return "station"; // typ objektu pre stanicu
}

bool StationObject::contains(double x, double y) const // kontrola ci je bod v stanici
{
    double dx = x - x_;
    double dy = y - y_;
    return std::sqrt(dx * dx + dy * dy) <= radius_; // bod je v stanici ak je vzdialenost mensia ako polomer
}

CircleObstacleObject::CircleObstacleObject(double x, double y, double radius)
    : GameObject(x, y, radius) // kruhova prekazka dedi z GameObject
{
}

std::string CircleObstacleObject::getObjectType() const
{
    return "circle_obstacle"; // typ objektu pre kruhovu prekazku
}

bool CircleObstacleObject::contains(double x, double y) const // kontrola ci je bod v kruhovej prekazke
{
    double dx = x - x_;
    double dy = y - y_;
    return std::sqrt(dx * dx + dy * dy) <= radius_; // ak je vzdialenost od stredu mensia ako radius, bod je v prekazke
}

RectangleObstacleObject::RectangleObstacleObject(
    double x,
    double y,
    double width,
    double height)
    : GameObject(x, y, 0.0), // obdlznik nema radius, preto 0.0
      width_(width),
      height_(height)
{
}

std::string RectangleObstacleObject::getObjectType() const
{
    return "rectangle_obstacle"; // typ objektu pre obdlznikovu prekazku
}

bool RectangleObstacleObject::contains(double x, double y) const // kontrola ci je bod v obdlzniku
{
    return x >= x_ - width_ * 0.5 && // lava hrana
           x <= x_ + width_ * 0.5 && // prava hrana
           y >= y_ - height_ * 0.5 && // dolna/hranicna cast
           y <= y_ + height_ * 0.5; // horna/hranicna cast
}

double RectangleObstacleObject::getWidth() const
{
    return width_; // vrati sirku obdlznika
}

double RectangleObstacleObject::getHeight() const
{
    return height_; // vrati vysku obdlznika
}

GameLogic::GameLogic(const GameConfig& config)
    : config_(config), // ulozenie konfiguracie z yaml
      rng_(std::random_device{}()) // inicializacia generatora nahodnych cisel
{
    validateConfig(); // kontrola ci su parametre v konfiguracii spravne

    environment::Config env_config;
    env_config.map_filename = config_.map_path; // cesta k png mape
    env_config.resolution = config_.map_resolution; // rozlisenie mapy

    env_ = std::make_shared<environment::Environment>(env_config); // vytvorenie prostredia zo zadania 1

    station_ = std::make_unique<StationObject>( // vytvorenie stanice
        config_.station_x,
        config_.station_y,
        config_.station_radius
    );

    loadObstacles(); // nacitanie geometrickych prekazok z konfiguracie

    player1_.max_capacity = config_.max_capacity; // nastavenie max kapacity pre hraca 1
    player2_.max_capacity = config_.max_capacity; // nastavenie max kapacity pre hraca 2

    generateTrash(); // vygenerovanie odpadkov na zaciatku hry
}

void GameLogic::validateConfig() const // kontrola konfiguracie
{
    if (config_.map_resolution <= 0.0) {
        throw GameConfigException("map_resolution must be positive."); // rozlisenie mapy musi byt kladne
    }

    if (config_.max_capacity <= 0) {
        throw GameConfigException("max_capacity must be positive."); // kapacita musi byt kladna
    }

    if (config_.trash_count <= 0) {
        throw GameConfigException("trash_count must be positive."); // pocet odpadkov musi byt kladny
    }

    if (config_.trash_radius_min <= 0.0) {
        throw GameConfigException("trash_radius_min must be positive."); // minimalny polomer musi byt kladny
    }

    if (config_.trash_radius_max < config_.trash_radius_min) {
        throw GameConfigException("trash_radius_max must be >= trash_radius_min."); // max polomer nemoze byt mensi ako min
    }

    if (config_.trash_types.empty()) {
        throw GameConfigException("trash_types cannot be empty."); // musime mat aspon jeden typ odpadu
    }

    if (config_.collect_distance <= 0.0) {
        throw GameConfigException("collect_distance must be positive."); // vzdialenost zbierania musi byt kladna
    }

    if (config_.station_radius <= 0.0) {
        throw GameConfigException("station_radius must be positive."); // polomer stanice musi byt kladny
    }

    if (config_.circle_obstacles_x.size() != config_.circle_obstacles_y.size() ||
        config_.circle_obstacles_x.size() != config_.circle_obstacles_radius.size()) {
        throw GameConfigException("Circle obstacle arrays must have the same size."); // polia kruhovych prekazok musia mat rovnaku velkost
    }

    if (config_.rectangle_obstacles_x.size() != config_.rectangle_obstacles_y.size() ||
        config_.rectangle_obstacles_x.size() != config_.rectangle_obstacles_width.size() ||
        config_.rectangle_obstacles_x.size() != config_.rectangle_obstacles_height.size()) {
        throw GameConfigException("Rectangle obstacle arrays must have the same size."); // polia obdlznikovych prekazok musia mat rovnaku velkost
    }
}

void GameLogic::loadObstacles() // vytvorenie prekazok z parametrov
{
    obstacles_.clear(); // najprv vymazeme stare prekazky

    for (size_t i = 0; i < config_.circle_obstacles_x.size(); ++i) { // prejdenie vsetkych kruhovych prekazok
        if (config_.circle_obstacles_radius[i] <= 0.0) {
            throw GameConfigException("Circle obstacle radius must be positive."); // radius musi byt kladny
        }

        obstacles_.push_back(
            std::make_unique<CircleObstacleObject>( // vytvorenie kruhovej prekazky
                config_.circle_obstacles_x[i],
                config_.circle_obstacles_y[i],
                config_.circle_obstacles_radius[i]
            )
        );
    }

    for (size_t i = 0; i < config_.rectangle_obstacles_x.size(); ++i) { // prejdenie vsetkych obdlznikovych prekazok
        if (config_.rectangle_obstacles_width[i] <= 0.0 ||
            config_.rectangle_obstacles_height[i] <= 0.0) {
            throw GameConfigException("Rectangle obstacle dimensions must be positive."); // sirka a vyska musia byt kladne
        }

        obstacles_.push_back(
            std::make_unique<RectangleObstacleObject>( // vytvorenie obdlznikovej prekazky
                config_.rectangle_obstacles_x[i],
                config_.rectangle_obstacles_y[i],
                config_.rectangle_obstacles_width[i],
                config_.rectangle_obstacles_height[i]
            )
        );
    }
}

void GameLogic::reset() // reset hry cez service
{
    player1_ = PlayerState{}; // vynulovanie stavu hraca 1
    player2_ = PlayerState{}; // vynulovanie stavu hraca 2

    player1_.max_capacity = config_.max_capacity; // znovu nastavime kapacitu hraca 1
    player2_.max_capacity = config_.max_capacity; // znovu nastavime kapacitu hraca 2

    game_finished_ = false; // hra znovu bezi

    generateTrash(); // vygenerujeme nove odpadky
}

void GameLogic::updatePlayer1Position(double x, double y) // aktualizacia polohy hraca 1 z odometrie
{
    player1_.x = x;
    player1_.y = y;
    player1_.has_odom = true; // uz mame platnu odometriu

    addPathPoint(player1_, x, y); // ulozenie bodu do prejdenej drahy
}

void GameLogic::updatePlayer2Position(double x, double y) // aktualizacia polohy hraca 2 z odometrie
{
    player2_.x = x;
    player2_.y = y;
    player2_.has_odom = true; // uz mame platnu odometriu

    addPathPoint(player2_, x, y); // ulozenie bodu do prejdenej drahy
}

void GameLogic::addPathPoint(PlayerState& player, double x, double y) // pridavanie bodov do prejdenej drahy
{
    if (player.path_x.empty()) { // ak je draha prazdna, ulozime prvy bod
        player.path_x.push_back(x);
        player.path_y.push_back(y);
        return;
    }

    double last_x = player.path_x.back(); // posledny ulozeny x bod drahy
    double last_y = player.path_y.back(); // posledny ulozeny y bod drahy

    if (distance(last_x, last_y, x, y) >= config_.path_min_step) { // novy bod ulozime len ak sa robot posunul dostatocne
        player.path_x.push_back(x);
        player.path_y.push_back(y);
    }
}

void GameLogic::generateTrash() // generator odpadkov
{
    trash_.clear(); // vymazanie starych odpadkov

    std::uniform_real_distribution<double> x_dist(0.0, env_->getWidth()); // nahodne x v rozsahu mapy
    std::uniform_real_distribution<double> y_dist(0.0, env_->getHeight()); // nahodne y v rozsahu mapy
    std::uniform_real_distribution<double> radius_dist(
        config_.trash_radius_min,
        config_.trash_radius_max
    ); // nahodny polomer odpadu medzi min a max

    int id = 0;
    int attempts = 0;
    const int max_attempts = config_.trash_count * 1000; // ochrana aby generator nebezel donekonecna

    while (id < config_.trash_count && attempts < max_attempts) { // generujeme kym nemame pozadovany pocet odpadkov
        attempts++;

        double x = x_dist(rng_); // nahodna x pozicia
        double y = y_dist(rng_); // nahodna y pozicia

        if (!isValidSpawnPosition(x, y)) { // ak pozicia nie je vhodna, skusime dalsiu
            continue;
        }

        std::string type = config_.trash_types[
            static_cast<size_t>(id) % config_.trash_types.size()
        ]; // typ odpadu sa vybera z konfiguracie cyklicky

        double radius = radius_dist(rng_); // nahodny polomer odpadu

        trash_.push_back(
            TrashFactory::createTrash(id, type, x, y, radius) // vytvorenie odpadu cez Factory
        );

        id++;
    }

    if (id < config_.trash_count) {
        throw GameConfigException("Could not generate requested number of trash objects."); // ak sa nepodarilo vygenerovat dost odpadu
    }
}

bool GameLogic::isInsideAnyObstacle(double x, double y) const // kontrola ci je bod v nejakej geometrickej prekazke
{
    for (const auto& obstacle : obstacles_) {
        if (obstacle->contains(x, y)) { // polymorfne volanie contains pre kruh/obdlznik/stanicu
            return true;
        }
    }

    return false;
}

bool GameLogic::isValidSpawnPosition(double x, double y) const // kontrola ci moze byt odpad na tejto pozicii
{
    if (env_->isOccupied(x, y)) { // nesmie byt v ciernej casti mapy
        return false;
    }

    if (isInsideAnyObstacle(x, y)) { // nesmie byt v geometrickej prekazke
        return false;
    }

    if (distance(x, y, config_.station_x, config_.station_y) <
        config_.station_radius + 1.0) { // nesmie byt prilis blizko stanice
        return false;
    }

    for (const auto& item : trash_) { // nesmie byt prilis blizko ineho odpadu
        if (distance(x, y, item.x, item.y) < 1.0) {
            return false;
        }
    }

    const double safety = config_.trash_radius_max + 0.15; // bezpecnostna rezerva okolo odpadu

    if (env_->isOccupied(x + safety, y)) return false; // kontrola okolia odpadu vpravo
    if (env_->isOccupied(x - safety, y)) return false; // kontrola okolia odpadu vlavo
    if (env_->isOccupied(x, y + safety)) return false; // kontrola okolia odpadu hore
    if (env_->isOccupied(x, y - safety)) return false; // kontrola okolia odpadu dole

    if (isInsideAnyObstacle(x + safety, y)) return false; // rezerva od geometrickych prekazok
    if (isInsideAnyObstacle(x - safety, y)) return false;
    if (isInsideAnyObstacle(x, y + safety)) return false;
    if (isInsideAnyObstacle(x, y - safety)) return false;

    return true; // pozicia je vhodna na spawn odpadu
}

void GameLogic::update() // hlavna aktualizacia hry
{
    if (game_finished_) { // ak hra skoncila, uz nic neaktualizujeme
        return;
    }

    if (player1_.has_odom) { // ak mame polohu hraca 1
        handlePlayer(player1_);
    }

    if (player2_.has_odom) { // ak mame polohu hraca 2
        handlePlayer(player2_);
    }

    checkGameFinished(); // kontrola ci uz hra neskoncila
}

void GameLogic::handlePlayer(PlayerState& player) // spracovanie hraca v jednom kroku hry
{
    collectTrash(player); // pokus o pozbieranie odpadu
    unloadAtStation(player); // pokus o vylozenie odpadu v stanici
}

void GameLogic::collectTrash(PlayerState& player) // zber odpadu
{
    if (player.current_capacity >= player.max_capacity) { // ak je kapacita plna, uz nezbiera
        return;
    }

    for (auto& item : trash_) { // prejdeme vsetky odpadky
        if (item.collected) { // uz pozbierane preskocime
            continue;
        }

        double d = distance(player.x, player.y, item.x, item.y); // vzdialenost hraca od odpadu

        if (d <= config_.collect_distance + item.radius) { // ak je robot dost blizko k odpadu
            item.collected = true; // oznacime odpad ako pozbierany
            player.current_capacity++; // zvysime aktualnu kapacitu

            if (item.type == "paper") { // zvysenie poctu podla typu odpadu
                player.paper_count++;
            } else if (item.type == "plastic") {
                player.plastic_count++;
            } else if (item.type == "glass") {
                player.glass_count++;
            }

            return; // naraz zbierame iba jeden odpad
        }
    }
}

void GameLogic::unloadAtStation(PlayerState& player) // vylozenie odpadu v stanici
{
    if (player.current_capacity <= 0) { // ak robot nic nenesie, nie je co vylozit
        return;
    }

    double d = distance(player.x, player.y, config_.station_x, config_.station_y); // vzdialenost od stanice

    if (d <= config_.station_radius) { // ak je robot v stanici
        player.score += player.current_capacity; // skore sa zvysi o pocet nesenych odpadkov
        player.current_capacity = 0; // robot vylozil vsetok odpad
    }
}

void GameLogic::checkGameFinished() // kontrola konca hry
{
    if (getRemainingTrashCount() > 0) { // ak je este odpad na ploche, hra pokracuje
        return;
    }

    if (player1_.current_capacity > 0 || player2_.current_capacity > 0) { // ak este niekto nesie odpad, musi ho vylozit
        return;
    }

    game_finished_ = true; // vsetko je pozbierane aj vylozene, hra skoncila
}

int GameLogic::getRemainingTrashCount() const // pocet nepozbieranych odpadkov
{
    int remaining = 0;

    for (const auto& item : trash_) { // prejdeme vsetky odpadky
        if (!item.collected) { // ak nie je pozbierany
            remaining++;
        }
    }

    return remaining;
}

std::string GameLogic::getGameStatus() const // textovy stav hry
{
    if (!game_finished_) {
        return "RUNNING"; // hra este bezi
    }

    if (player1_.score > player2_.score) {
        return "PLAYER 1 WINS!"; // vyhral hrac 1
    }

    if (player2_.score > player1_.score) {
        return "PLAYER 2 WINS!"; // vyhral hrac 2
    }

    return "DRAW!"; // remiza
}

GameSnapshot GameLogic::getSnapshot() const // vytvori kopiu aktualneho stavu hry pre ROS wrapper
{
    GameSnapshot snapshot;

    snapshot.player1 = player1_; // stav hraca 1
    snapshot.player2 = player2_; // stav hraca 2

    snapshot.trash = trash_; // aktualne odpadky

    snapshot.remaining_trash = getRemainingTrashCount(); // pocet zostavajucich odpadkov

    snapshot.game_finished = game_finished_; // ci hra skoncila
    snapshot.status = getGameStatus(); // textovy stav hry

    snapshot.station_x = config_.station_x; // poloha stanice x
    snapshot.station_y = config_.station_y; // poloha stanice y
    snapshot.station_radius = config_.station_radius; // polomer stanice

    snapshot.circle_obstacles_x = config_.circle_obstacles_x; // kruhove prekazky do stavu hry
    snapshot.circle_obstacles_y = config_.circle_obstacles_y;
    snapshot.circle_obstacles_radius = config_.circle_obstacles_radius;

    snapshot.rectangle_obstacles_x = config_.rectangle_obstacles_x; // obdlznikove prekazky do stavu hry
    snapshot.rectangle_obstacles_y = config_.rectangle_obstacles_y;
    snapshot.rectangle_obstacles_width = config_.rectangle_obstacles_width;
    snapshot.rectangle_obstacles_height = config_.rectangle_obstacles_height;

    return snapshot; // vrati aktualny stav hry
}

double GameLogic::distance(double x1, double y1, double x2, double y2) const // euklidovska vzdialenost dvoch bodov
{
    double dx = x1 - x2;
    double dy = y1 - y2;
    return std::sqrt(dx * dx + dy * dy);
}

} // namespace game