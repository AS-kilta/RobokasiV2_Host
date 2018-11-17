//
// Created by lehdari on 13.10.2018.
//

#include "Joint.hpp"


using namespace kin;


Joint::Joint(const DHMatrix& dh, const Mat4f& base) :
    _dh     (dh),
    _bm     (base),
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
    _dh.setTheta(angle);
    _dirty = true;
}

float Joint::getAngle() const
{
    return _dh.getTheta();
}

const DHMatrix& Joint::getDHMatrix(void) const
{
    return _dh;
}

const Mat4f &Joint::getEnd()
{
    if (_dirty) {
        _end = _bm * _dh.getMatrix();
        _dirty = false;
    }

    return _end;
}
