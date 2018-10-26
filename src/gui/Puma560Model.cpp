//
// Created by lehdari on 21.10.2018.
//

#include "Puma560Model.hpp"


using namespace gui;


Mesh Puma560Model::_links[6];


Puma560Model::Puma560Model()
{
    _links[0].loadFromObj(std::string(RES_PATH) + "models/puma_link1_mod.obj");
    _links[1].loadFromObj(std::string(RES_PATH) + "models/puma_link2_mod.obj");
    _links[2].loadFromObj(std::string(RES_PATH) + "models/puma_link3_mod.obj");
    _links[3].loadFromObj(std::string(RES_PATH) + "models/puma_link4_mod.obj");
    _links[4].loadFromObj(std::string(RES_PATH) + "models/puma_link5_mod.obj");
    _links[5].loadFromObj(std::string(RES_PATH) + "models/puma_link6_mod.obj");
}

Puma560Model::~Puma560Model()
{ }

void Puma560Model::render(Shader& shader, Camera& camera)
{
    for (auto i=0; i<6; ++i) {
        _links[i].render(shader, camera, _chain.getJointEnd(i));
    }
}
