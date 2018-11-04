//
// Created by lehdari on 13.10.2018.
//

#include "DHMatrix.hpp"


using namespace kin;


DHMatrix::DHMatrix(float d, float theta, float r, float alpha) :
    _d      (d),
    _theta  (theta),
    _r      (r),
    _alpha  (alpha),
    _x      (Mat4f::Identity()),
    _xDirty (true),
    _z      (Mat4f::Identity()),
    _zDirty (true),
    _m      (Mat4f::Identity())
{
    update();
}

void DHMatrix::setTheta(float theta)
{
    _theta = theta;
    _zDirty = true;
}

float DHMatrix::getTheta() const
{
    return _theta;
}

DHMatrix::operator const Mat4f&()
{
    if (_zDirty || _xDirty)
        update();
    return _m;
}

const Mat4f& DHMatrix::getMatrix()
{
    if (_zDirty || _xDirty)
        update();
    return _m;
}

const Mat4f& DHMatrix::getLinkMatrix()
{
    if (_xDirty)
        update();
    return _x;
}

const Mat4f& DHMatrix::getJointMatrix()
{
    if (_zDirty)
        update();
    return _z;
}

void DHMatrix::update()
{
    if (_zDirty) {
        float st = std::sin(_theta);
        float ct = std::cos(_theta);
        _z  <<  ct,     -st,    0.0f,   0.0f,
                st,     ct,     0.0f,   0.0f,
                0.0f,   0.0f,   1.0f,   _d,
                0.0f,   0.0f,   0.0f,   1.0f;
        _zDirty = false;
    }
    if (_xDirty) {
        float sa = std::sin(_alpha);
        float ca = std::cos(_alpha);
        _x  <<  1.0f,   0.0f,   0.0f,   _r,
                0.0f,   ca,     -sa,    0.0f,
                0.0f,   sa,     ca,     0.0f,
                0.0f,   0.0f,   0.0f,   1.0f;
        _xDirty = false;
    }
    _m = _z * _x;
}
