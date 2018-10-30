//
// Created by lehdari on 20.10.2018.
//

#ifndef ROBOKASIV2_HOST_RENDERER_HPP
#define ROBOKASIV2_HOST_RENDERER_HPP


#include <memory>
#include <vector>

#include "Shader.hpp"
#include "Camera.hpp"
#include "Renderable.hpp"


namespace gui {

    class Renderer {
    public:
        Renderer(Shader& shader, Camera& camera);

        void render(const std::vector<std::shared_ptr<Renderable>>& renderables);

    private:
        Shader&  _shader;
        Camera&  _camera;
    };

} // namespace gui


#endif //ROBOKASIV2_HOST_RENDERER_HPP
