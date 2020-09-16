//
// Created by abiel on 9/15/20.
//

#ifndef MICRODRONITESM_GUI_DRAWABLESWARM_H
#define MICRODRONITESM_GUI_DRAWABLESWARM_H

#include "SFML/Graphics.hpp"
#include "SimulatedSwarm.h"

class DrawableSwarm : public sf::Drawable, public SimulatedSwarm {
public:
    void draw(sf::RenderTarget &target, sf::RenderStates states) const override;

private:
    const double scaleFactor = 100.0;
};


#endif //MICRODRONITESM_GUI_DRAWABLESWARM_H
