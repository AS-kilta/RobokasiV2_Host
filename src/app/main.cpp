//
// Created by Lehdari on 2018-10-10.
//

#include "SDLApp.hpp"
#include <kinematics/Puma560.hpp>
#include <hwio/Hello.hpp>
#include <iostream>


int main(int argv, char** args)
{
    try {
        hwio::Hello hwioHello;
        hwioHello.hello();

        SDLApp app(SDLApp::Settings(SDLApp::WindowSettings("Robokasi")));
        app.loop();
    } catch (const char* msg) {
        fprintf(stderr, "Error: %s\n", msg);
        return 1;
    }

    return 0;
}
