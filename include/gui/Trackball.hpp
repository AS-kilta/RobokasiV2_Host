#ifndef ROBOKASIV2_HOST_GUI_TRACKBALL_HPP
#define ROBOKASIV2_HOST_GUI_TRACKBALL_HPP


#include "Camera.hpp"
#include "kinematics/MathTypes.hpp"


namespace gui {

    struct Trackball {
        float theta = PI * 0.75f;
        float phi = PI * 0.25f;
        float r = 2500.0f;
        float speed = 0.01;
        void handleMouseMove(int dx, int dy);
        void handleMouseScroll(int d);
        void doLookAt(Camera& camera);
    };

}


#endif
