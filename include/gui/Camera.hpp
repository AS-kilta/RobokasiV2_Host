//
// Created by lehdari on 14.10.2018.
//

#ifndef ROBOKASIV2_HOST_GUI_CAMERA_HPP
#define ROBOKASIV2_HOST_GUI_CAMERA_HPP


#include "GLTypes.hpp"


namespace gui {

    class Camera {
    public:
        Camera(void);

        void lookAt(const Vec3GLf& from, const Vec3GLf& to, const Vec3GLf& up);

        void projection(float fov, float aspectRatio, float near, float far);

        Mat4GLf getVP(void) const;

    private:
        Mat4GLf _orientation;
        Mat4GLf _projection;
    };

} // namespace gui


#endif //ROBOKASIV2_HOST_GUI_CAMERA_HPP
