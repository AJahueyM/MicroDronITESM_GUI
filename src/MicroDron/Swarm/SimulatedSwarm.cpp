//
// Created by abiel on 9/15/20.
//

#include "SimulatedSwarm.h"
#include <fmt/format.h>

SimulatedSwarm::SimulatedSwarm() {
    //Initialize swarm with 3 follower drones and 9 followers
    DronePos pos;

    drones.emplace_back(DroneData(DroneType::Leader1, pos));
    pos.x += 1;
    drones.emplace_back(DroneData(DroneType::Leader2, pos));
    pos.x -= 0.5; pos.y += 1.0;
    drones.emplace_back(DroneData(DroneType::Leader3, pos));

    pos.y -= 0.5; pos.z += 1.0;
    drones.emplace_back(DroneData(DroneType::Leader4, pos));

    pos = DronePos(0, 1, 0);
    addFollowers(3, pos, 1);
}

void SimulatedSwarm::addFollowers(size_t amount, const DronePos &origin, double spacing) {
    auto pos = origin;

    for(size_t i = 0; i < amount; ++i){
        for(size_t j = 0; j < amount; ++j){
            drones.emplace_back(DroneData(DroneType::Follower, pos));
            pos.x += spacing;
        }
        pos.y += spacing;
        pos.x = origin.x;
    }
}

double SimulatedSwarm::getDistance(size_t id1, size_t id2) const {
    auto pos1 = drones.at(id1).pos;
    auto pos2 = drones.at(id2).pos;

    return std::sqrt(
            std::pow(pos1.x - pos2.x, 2.0) +
            std::pow(pos1.y - pos2.y, 2.0) +
            std::pow(pos1.z - pos2.z, 2.0));
}

double SimulatedSwarm::getDistance(size_t id, DroneType type) const {
    auto it = std::find_if(drones.begin(), drones.end(),
                           [&] (const DroneData &d) {return d.type == type;});

    return getDistance(id, std::distance(drones.begin(), it));
}

void SimulatedSwarm::applyVelocities(const std::vector<DronePos> &velocities, double dt) {
    if(velocities.size() != drones.size()){
        throw std::runtime_error("Not enough velocities given");
    }

    for(size_t i = 0; i < drones.size(); ++i){
        applyVelocity(i, velocities.at(i), dt);
    }
}

void SimulatedSwarm::applyVelocity(size_t id, const DronePos &velocity, double dt) {
    auto drone = drones.at(id);
    drone.pos.x += velocity.x * dt;
    drone.pos.y += velocity.y * dt;
    drone.pos.z += velocity.z * dt;

    drones[id] = drone;
}

void SimulatedSwarm::moveDrones(const std::vector<DronePos> &deltaPosition) {
    if(deltaPosition.size() != drones.size()){
        throw std::runtime_error("Not enough velocities given");
    }

    for(size_t i = 0; i < drones.size(); ++i){
        moveDrone(i, deltaPosition.at(i));
    }
}

void SimulatedSwarm::moveDrone(size_t id, const DronePos &delta) {
    applyVelocity(id, delta, 1);
}

