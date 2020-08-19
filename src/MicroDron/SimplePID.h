//
// Created by abiel on 8/19/20.
//

#ifndef MICRODRONITESM_GUI_SIMPLEPID_H
#define MICRODRONITESM_GUI_SIMPLEPID_H

struct SimplePID{
    float p = 0;
    float i = 0;
    float d = 0;
    bool clamped = false;
    float maxOutput = 0;
    float minOutput = 0;
    bool continuous = false;
    float maxInput = 0;
    float minInput = 0;
};


#endif //MICRODRONITESM_GUI_SIMPLEPID_H
