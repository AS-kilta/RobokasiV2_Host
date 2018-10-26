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

        void lookAt(const Vec3GLf& eye, const Vec3GLf& target, const Vec3GLf& up);
        void projection(float fov, float aspectRatio, float near, float far);
        Mat4GLf getWorldToClip(void) const;

    private:
        Mat4GLf _worldToCamera;
        Mat4GLf _cameraToClip;
    };

} // namespace gui


#endif //ROBOKASIV2_HOST_GUI_CAMERA_HPP
