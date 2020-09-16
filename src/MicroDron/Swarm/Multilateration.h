//
// Created by abiel on 9/16/20.
//

#ifndef MICRODRONITESM_GUI_MULTILATERATION_H
#define MICRODRONITESM_GUI_MULTILATERATION_H

//https://www.researchgate.net/profile/Abdelmoumen_Norrdine/publication/275027725_An_Algebraic_Solution_to_the_Multilateration_Problem/links/552f78c80cf22d437170e3c1.pdf

#include "Pose3D.h"
#include <vector>

struct ReferenceMeasurement {
    Pose3D referencePosition;
    double distance{};
};

namespace Multilateration {
    /**
     * Solves 3D multilateration given 3 or more reference measurements
     * @param measurments
     * @return
     */
    Pose3D solveMultilateration(const std::vector<ReferenceMeasurement> &measurments);
};


#endif //MICRODRONITESM_GUI_MULTILATERATION_H
