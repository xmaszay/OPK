#include "robot/Robot.h"

#include <cmath>

namespace robot {

Robot::Robot(const Config& config, const CollisionCb& collision_cb, bool start_thread)
    : config_(config),
      collision_cb_(collision_cb),
      running_(start_thread),
      in_collision_(false),
      start_thread_(start_thread) {

    state_.x = 0.0; 
    state_.y = 0.0;
    state_.theta = 0.0;
    state_.velocity = {0.0, 0.0};

    target_velocity_ = {0.0, 0.0};
    last_command_time_ = std::chrono::steady_clock::now(); // ulozi aktualny cas  

    if (start_thread_) { // ak je povolene vlakno spusti sa simulation loop
    worker_ = std::thread(&Robot::simulationLoop, this);
    }                  
}

Robot::~Robot() {
    running_ = false;

    if (worker_.joinable()) { // ci je mozne nad vlakno zavolat join
        worker_.join(); // cakanie na koniec vlakna 
    }
}

void Robot::setVelocity(const geometry::Twist& velocity) {
    std::lock_guard<std::mutex> lock(mutex_); // zamkne mutex 
    target_velocity_ = velocity; // ulozi novy prikaz
    last_command_time_ = std::chrono::steady_clock::now(); // obnovi cas posledneho prikazu
} // cize po tomto vieme aku rychlost chceme a aky prikaz je este platny 

geometry::RobotState Robot::getState() const { // zamkne mutex a vrati aktualny stav
    std::lock_guard<std::mutex> lock(mutex_);  // chrani predtym aby sa cital stav, ked ho vlakno meni 
    return state_;
}

bool Robot::isInCollision() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return in_collision_; // vracia spat collision
}

void Robot::update(const geometry::Twist& velocity, double dt) { 
    auto now = std::chrono::steady_clock::now(); // zistujeme ci expiroval prikaz
    double elapsed = std::chrono::duration<double>(now - last_command_time_).count();// spocita kolko casu preslo od posledneho setVelocity

    bool emergency_braking = (elapsed > config_.command_duration) && (velocity.linear == 0.0) && (velocity.angular == 0.0); // ak prikaz vyprsal, rychlost je nulova, bude brzdit

    auto updateAxis = [dt](double current, double target, double acceleration) { // postupne priblizovanie sa aktualnej rychlosti k cielovej
        if (current < target) { // ak je aktualna mensia ako cielova, zrychli 
            current += acceleration * dt;
            if (current > target) {
                current = target;
            }
        } else if (current > target) {
            current -= acceleration * dt;
            if (current < target) {
                current = target;
            }
        }
        return current;
    };

    double linear_acc = emergency_braking // vybera spravne zrychlenie 
        ? config_.emergency_decelerations.linear 
        : config_.accelerations.linear;  // pouziva sa , ale po expiracii prikazu sa pouzije deceleration

    double angular_acc = emergency_braking
        ? config_.emergency_decelerations.angular
        : config_.accelerations.angular;

    state_.velocity.linear = updateAxis( // aktualizuje linearnu rychlost 
        state_.velocity.linear,
        velocity.linear,
        linear_acc
    );

    state_.velocity.angular = updateAxis( // aktualizuje angularnu rychlost 
        state_.velocity.angular,
        velocity.angular,
        angular_acc
    );

    double new_theta = state_.theta + state_.velocity.angular * dt; // najprv zmenime orientaciu 
    double new_x = state_.x + state_.velocity.linear * dt * std::cos(new_theta); // podla novej orientacie vypocitame pohyb v priestore
    double new_y = state_.y + state_.velocity.linear * dt * std::sin(new_theta);

geometry::RobotState new_state = state_;
new_state.x = new_x;
new_state.y = new_y;
new_state.theta = new_theta;

if (collision_cb_ && collision_cb_(new_state)) { // ak callback povie ze novy stav je v kolizii
    // kolizia, zastav linear pohyb
    state_.velocity.linear = 0.0; // vynuluje sa linearna rychlost
    in_collision_ = true;
} else {
    state_ = new_state;
    in_collision_ = false;
}
}

void Robot::simulationLoop() {
    const double dt = static_cast<double>(config_.simulation_period_ms) / 1000.0; // prevod ms na s

    while (running_) {
        geometry::Twist command_to_apply; // prikaz ktory sa ma aplikovat

        {
            std::lock_guard<std::mutex> lock(mutex_);

            auto now = std::chrono::steady_clock::now(); // zisti aktualny cas
            double elapsed = std::chrono::duration<double>(now - last_command_time_).count(); // cas od posledneho prikazu v sekundach 

            if (elapsed > config_.command_duration) { // ak cas od posledneho prikazu presiahol command duration nastavi:
                command_to_apply.linear = 0.0;
                command_to_apply.angular = 0.0;
            } else {
                command_to_apply = target_velocity_;
            }

            update(command_to_apply, dt); // robot sa pohne 
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(config_.simulation_period_ms)); // caka do dalsieho kroku 
    }
}

void Robot::setState(const geometry::RobotState& state) { // rucne nastavi stav robota, napr na start v strede mapy
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = state;
}

} // namespace robot