//
// Created by lehdari on 13.10.2018.
//

#include "Joint.hpp"


using namespace kin;


Joint::Joint(const DHMatrix& dh, const Mat4f& base) :
    _dh     (dh),
    _bm     (base),
    _a      (0.0f),
    _rm     (Mat4f::Identity()),
    _end    (Mat4f::Identity()),
    _dirty  (true)
{
}

void Joint::setBase(const Mat4f& base)
{
    _bm = base;
    _dirty = true;
}

void Joint::setAngle(float angle)
{
    _a = angle;
    float sa = std::sin(_a);
    float ca = std::cos(_a);
    _rm <<  ca,     -sa,    0.0f,   0.0f,
            sa,     ca,     0.0f,   0.0f,
            0.0f,   0.0f,   1.0f,   0.0f,
            0.0f,   0.0f,   0.0f,   1.0f;
    _dirty = true;
}

const DHMatrix& Joint::getDHMatrix(void) const
{
    return _dh;
}

const Mat4f &Joint::getEnd()
{
    if (_dirty) {
        _end = _bm * _rm * _dh.getMatrix();
        _dirty = false;
    }

    return _end;
}
