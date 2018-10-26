//
// Created by lehdari on 14.10.2018.
//

#include "Camera.hpp"


using namespace gui;


Camera::Camera(void) :
    _worldToCamera(Mat4GLf::Identity()),
    _cameraToClip(Mat4GLf::Identity())
{}

void Camera::lookAt(const Vec3GLf& eye, const Vec3GLf& target, const Vec3GLf& up) {
    // Right handed coordinate system
    Vec3GLf zAxis = (eye - target).normalized();
    Vec3GLf xAxis = up.cross(zAxis).normalized();
    Vec3GLf yAxis = zAxis.cross(xAxis);

    _worldToCamera << xAxis[0], xAxis[1], xAxis[2], -xAxis.dot(eye),
                      yAxis[0], yAxis[1], yAxis[2], -yAxis.dot(eye),
                      zAxis[0], zAxis[1], zAxis[2], -zAxis.dot(eye),
                      0.0f    , 0.0f    , 0.0f    , 1.0f           ;
}

void Camera::projection(float fov, float aspectRatio, float near, float far) {
    float r = tanf(fov / 2.0f) * near;
    float t = r / aspectRatio;

    _cameraToClip << near/r, 0.0f  , 0.0f                  , 0.0f                     ,
                     0.0f  , near/t, 0.0f                  , 0.0f                     ,
                     0.0f  , 0.0f  , -(far+near)/(far-near), -2.0f*far*near/(far-near),
                     0.0f  , 0.0f  , -1.0f                 , 0.0f                     ;
}

Mat4GLf Camera::getWorldToClip(void) const {
    return _cameraToClip * _worldToCamera;
}
