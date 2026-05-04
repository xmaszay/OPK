#pragma once

#include <functional>
#include <thread> // vlakna pouzivame kvoli realtime control 
#include <mutex>
#include <atomic>
#include <chrono>
#include "types/Geometry.h"

namespace robot {

struct Config { // konfiguracna struktura robota
    geometry::Twist accelerations; // linear, angular
    geometry::Twist emergency_decelerations;
    double command_duration; // dlzka platnosti prikazu, ak nepride dalsi prikaz robot zacne brzdit 
    int simulation_period_ms; // frekvencia vykonavania simulacie vo vlakne
};


class Robot {
public:
    using CollisionCb = std::function<bool(geometry::RobotState)>; // pre bonus, typ cb funkcie pre koliziu 

    Robot(const Config& config, const CollisionCb& collision_cb = nullptr, bool start_thread = true); // konstruktor, true alebo false mame kvoli testom, false aby sa robot sam nehybal vo vlakne
    ~Robot(); // destruktor
    void setVelocity(const geometry::Twist& velocity);
    void setState(const geometry::RobotState& state); // nastavenie robota rucne, napr na nastavenie pociatocnej pozicie
    geometry::RobotState getState() const; // vracia aktualny stav robota
    bool isInCollision() const; // vracia ci je robot prave v kolizii

private:
    void simulationLoop(); // funkcia ktora bezi vo vlakne a pravidelne aktualizuje stav robota

    Config config_; // config robota
    CollisionCb collision_cb_; // callback fun pre koliziu 

    geometry::RobotState state_;    // aktualny stav robota
    geometry::Twist target_velocity_; // pozadovana rychlost ktoru som naposledy poslal cez setVelocity

    mutable std::mutex mutex_; // na ochranu zdielanych premennych  
    std::thread worker_; // simualcny thread
    std::atomic<bool> running_; // hovori nam ci ma vlakno este bezat

    bool in_collision_; 
    std::chrono::steady_clock::time_point last_command_time_; // cas posledneho prikazu

    bool start_thread_; // uklada ci sa ma spustit thread

protected: // aby sme vedeli testovat cez odvodenu triedu, cize vie dedit 
    void update(const geometry::Twist& velocity, double dt); // aktualizuje stav robota

};
} // namespace robot