//
// Created by abiel on 9/15/20.
//

#ifndef MICRODRONITESM_GUI_SIMULATEDSWARM_H
#define MICRODRONITESM_GUI_SIMULATEDSWARM_H

#include <vector>
#include <string>

class DronePos {
public:
    DronePos(){
        ;
    }

    DronePos(double x, double y, double z){
        this->x = x; this->y = y; this->z = z;
    }

    double x{0}, y{0}, z{0};
};

enum class DroneType {
    Leader1,
    Leader2,
    Leader3,
    Follower,
    Undefined
};

class DroneData {
public:
    DroneData(){
        ;
    }

    DroneData(DroneType type, DronePos pos){
        this->type = type;
        this->pos = pos;
    }
    DroneType type = DroneType::Undefined;
    DronePos pos{};
};

class SimulatedSwarm {
public:
    SimulatedSwarm();

    [[nodiscard]] double getDistance(size_t id1, size_t id2) const;

    void applyVelocities(const std::vector<DronePos> &velocities, double dt);
    void applyVelocity(size_t id, const DronePos &velocity, double dt);

    void moveDrones(const std::vector<DronePos> &deltaPosition);
    void moveDrone(size_t id, const DronePos &delta);

    std::vector<DroneData> drones;

    //Draws a grid with amount ^ 2 of drones
    void addFollowers(size_t amount, const DronePos & origin, double spacing);
};


#endif //MICRODRONITESM_GUI_SIMULATEDSWARM_H
