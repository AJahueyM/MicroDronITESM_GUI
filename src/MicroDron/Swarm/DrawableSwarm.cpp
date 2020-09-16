//
// Created by abiel on 9/15/20.
//

#include "DrawableSwarm.h"

void DrawableSwarm::draw(sf::RenderTarget &target, sf::RenderStates states) const {
    for(const auto &drone : this->drones){
        sf::CircleShape circle(10);
        circle.setPosition(drone.pos.x * scaleFactor + 250, drone.pos.y * scaleFactor + 250);
        circle.setFillColor(drone.type != DroneType::Follower ? sf::Color::Red : sf::Color::Green);

        target.draw(circle);
    }
}