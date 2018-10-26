//
// Created by lehdari on 14.10.2018.
//

#ifndef ROBOKASIV2_HOST_GUI_RENDERABLE_HPP
#define ROBOKASIV2_HOST_GUI_RENDERABLE_HPP

#include "GLTypes.hpp"

namespace gui {

    class Shader;
    class Camera;

    class Renderable {
    public:
        Renderable(const Mat4GLf& objectToWorld);
        virtual ~Renderable() {};

        virtual void render(Shader& shader, Camera& camera) = 0;

    protected:
        Mat4GLf _objectToWorld;
    };

} // namespace gui


#endif //ROBOKASIV2_HOST_GUI_RENDERABLE_HPP
