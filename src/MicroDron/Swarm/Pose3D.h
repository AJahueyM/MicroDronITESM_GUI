//
// Created by abiel on 9/16/20.
//

#ifndef MICRODRONITESM_GUI_POSE3D_H
#define MICRODRONITESM_GUI_POSE3D_H

struct Pose3D {
    Pose3D(){;}

    Pose3D(double x, double y, double z){
        this->x = x;
        this->y = y;
        this->z = z;
    }

    double x{0}, y{0}, z{0};
};

#endif //MICRODRONITESM_GUI_POSE3D_H
