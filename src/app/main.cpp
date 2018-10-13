//
// Created by Lehdari on 2018-10-10.
//

#include "SDLApp.hpp"
#include <kinematics/Chain.hpp>
#include <gui/Hello.hpp>
#include <hwio/Hello.hpp>
#include <iostream>


int main(int argv, char** args) {

    gui::Hello guiHello;
    guiHello.hello();
    hwio::Hello hwioHello;
    hwioHello.hello();

    kin::Chain chain;
    chain.addJoint(kin::Joint(kin::DHMatrix(
    0.0f, 0.0f, 0.0f, PI*0.5f)));
    chain.addJoint(kin::Joint(kin::DHMatrix(
    0.0f, 0.0f, 0.4318f, 0.0f)));
    chain.addJoint(kin::Joint(kin::DHMatrix(
    0.15f, 0.0f, 0.0203f, -PI*0.5f)));
    chain.addJoint(kin::Joint(kin::DHMatrix(
    0.4318f, 0.0f, 0.0f, PI*0.5f)));
    chain.addJoint(kin::Joint(kin::DHMatrix(
    0.0f, 0.0f, 0.0f, -PI*0.5f)));
    chain.addJoint(kin::Joint(kin::DHMatrix(
    0.0f, 0.0f, 0.0f, 0.0f)));

    SDLApp app(SDLApp::Settings("Robokasi"));
    app.loop();

    return 0;
}
