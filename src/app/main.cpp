//
// Created by Lehdari on 2018-10-10.
//

#include "SDLApp.hpp"
#include <kinematics/Puma560.hpp>
#include <hwio/Hello.hpp>
#include <iostream>


int main(int argv, char** args)
{
    hwio::Hello hwioHello;
    hwioHello.hello();

    SDLApp app(SDLApp::Settings("Robokasi"));
    app.loop();

    return 0;
}
