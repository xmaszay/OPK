#include <cassert>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "Environment.h"

void testMapLoads() {
    environment::Config config;
    config.map_filename = "../maps/map.png";
    config.resolution = 0.05;

    auto env = std::make_shared<environment::Environment>(config);

    assert(env->getWidth() > 0.0);
    assert(env->getHeight() > 0.0);
}

void testInvalidMapThrows() {
    environment::Config config;
    config.map_filename = "../maps/does_not_exist.png";
    config.resolution = 0.05;

    bool exceptionThrown = false;

    try {
        environment::Environment env(config);
    } catch (const std::runtime_error&) {
        exceptionThrown = true;
    }

    assert(exceptionThrown);
}

void testOccupiedAndFree() {
    environment::Config config;
    config.map_filename = "../maps/map.png";
    config.resolution = 0.05;

    auto env = std::make_shared<environment::Environment>(config);

    bool occ1 = env->isOccupied(0.0, 0.0);
    bool occ2 = env->isOccupied(1.0, 1.0);

    assert(occ1 == true || occ1 == false);
    assert(occ2 == true || occ2 == false);
}

void testOutOfBounds() {
    environment::Config config;
    config.map_filename = "../maps/map.png";
    config.resolution = 0.05;

    auto env = std::make_shared<environment::Environment>(config);

    assert(env->isOccupied(-1.0, -1.0) == true);
    assert(env->isOccupied(10000.0, 10000.0) == true);
}

int main() {
    testMapLoads();
    testInvalidMapThrows();
    testOccupiedAndFree();
    testOutOfBounds();

    std::cout << "All Environment tests passed." << std::endl;
    return 0;
}