//
// Created by lehdari on 20.10.2018.
//

#include "Renderer.hpp"


using namespace gui;


Renderer::Renderer(Shader& shader, Camera& camera) :
    _shader (shader),
    _camera (camera)
{
}

void Renderer::render(const std::vector<std::shared_ptr<Renderable>>& renderables)
{
    _shader.use();
    for (auto& r : renderables) {
        r->render(_shader, _camera);
    }
}
