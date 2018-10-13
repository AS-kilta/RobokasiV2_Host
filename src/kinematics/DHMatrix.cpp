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
    _dirty  (true),
    _m      (Mat4f::Identity())

{
    update();
}

DHMatrix::operator const Mat4f&()
{
    if (_dirty)
        update();
    return _m;
}

const Mat4f& DHMatrix::getMatrix()
{
    if (_dirty)
        update();
    return _m;
}

void DHMatrix::update()
{
    float st = std::sin(_theta);
    float ct = std::cos(_theta);
    float sa = std::sin(_alpha);
    float ca = std::cos(_alpha);

    _z  <<  ct,     -st,    0.0f,   0.0f,
            st,     ct,     0.0f,   0.0f,
            0.0f,   0.0f,   1.0f,   _d,
            0.0f,   0.0f,   0.0f,   1.0f;
    _x  <<  1.0f,   0.0f,   0.0f,   _r,
            0.0f,   ca,     -sa,    0.0f,
            0.0f,   sa,     ca,     0.0f,
            0.0f,   0.0f,   0.0f,   1.0f;
    _m = _z * _x;

    _dirty = false;
}
