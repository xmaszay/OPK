#include <cassert>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "environment/Environment.h"

void testMapLoads() { // test na oberenie nacitania mapy, vytvorenie prostredia a nenulove sirka a vyska
    environment::Config config;
    config.map_filename = "../resources/opk-map.png";
    config.resolution = 0.05;

    auto env = std::make_shared<environment::Environment>(config);

    assert(env->getWidth() > 0.0); // assert overuje pravdivost podmienky ak nie vypise chybu
    assert(env->getHeight() > 0.0);
}

void testInvalidMapThrows() { //ak neexistuje mapa, vyhodi vynimku
    environment::Config config;
    config.map_filename = "../resources/does_not_exist.png";
    config.resolution = 0.05;

    bool exceptionThrown = false;

    try {
        environment::Environment env(config);
    } catch (const std::runtime_error&) {
        exceptionThrown = true;
    }

    assert(exceptionThrown);
}

void testOutOfBounds() { // overuje spravanie mimo mapy
    environment::Config config;
    config.map_filename = "../resources/opk-map.png";
    config.resolution = 0.05;

    auto env = std::make_shared<environment::Environment>(config);

    assert(env->isOccupied(-1.0, -1.0) == true);
    assert(env->isOccupied(10000.0, 10000.0) == true);
}

int main() {
    testMapLoads();
    testInvalidMapThrows();
    testOutOfBounds();

    std::cout << "All Environment tests passed." << std::endl;
    return 0;
}