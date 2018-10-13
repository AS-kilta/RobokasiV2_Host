//
// Created by lehdari on 14.10.2018.
//

#include "Camera.hpp"


using namespace gui;


Camera::Camera(void) :
    _orientation(Mat4GLf::Identity()),
    _projection(Mat4GLf::Identity())
{}

void Camera::lookAt(const Vec3GLf& from, const Vec3GLf& to, const Vec3GLf& up) {
    Vec3GLf zAxis = (to - from).normalized();
    Vec3GLf xAxis = up.cross(zAxis).normalized();
    Vec3GLf yAxis = zAxis.cross(xAxis);

    _orientation << xAxis[0]    , xAxis[1]  , xAxis[2]  , -xAxis.dot(from),
                    yAxis[0]    , yAxis[1]  , yAxis[2]  , -yAxis.dot(from),
                    -zAxis[0]   , -zAxis[1] , -zAxis[2] , zAxis.dot(from) ,
                    0.0f        , 0.0f      , 0.0f      , 1.0f;
}

void Camera::projection(float fov, float aspectRatio, float near, float far) {
    float r = tanf(fov / 2.0f) * near;
    float t = r / aspectRatio;

    _projection <<  near/r   , 0.0f      , 0.0f                  , 0.0f                      ,
                    0.0f     , near/t    , 0.0f                  , 0.0f                      ,
                    0.0f     , 0.0f      , -(far+near)/(far-near), -2.0f*far*near/(far-near) ,
                    0.0f     , 0.0f      , -1.0f                 , 0.0f                      ;
}

Mat4GLf Camera::getVP(void) const {
    return _projection * _orientation;
}
