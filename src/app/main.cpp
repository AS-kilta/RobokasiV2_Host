//
// Created by Lehdari on 2018-10-10.
//

#include "SDLApp.hpp"
#include <kinematics/Puma560.hpp>
#include <gui/Hello.hpp>
#include <hwio/Hello.hpp>
#include <iostream>


int main(int argv, char** args) {

    gui::Hello guiHello;
    guiHello.hello();
    hwio::Hello hwioHello;
    hwioHello.hello();

    kin::Puma560 arm;
    std::cout << arm.getEnd() << std::endl;

    for (auto i=0; i<10; ++i) {
        float theta = i*0.1*PI;
        arm.setJointAngle(5, theta);
        std::cout << arm.getEnd() << std::endl;
    }

    SDLApp app(SDLApp::Settings("Robokasi"));
    app.loop();

    return 0;
}
