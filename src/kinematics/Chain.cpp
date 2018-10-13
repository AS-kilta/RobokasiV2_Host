//
// Created by lehdari on 13.10.2018.
//

#include "Chain.hpp"
#include <cstdio>
#include <kinematics/Chain.hpp>


using namespace kin;


Chain::Chain(const Mat4f& base) :
    _bm         (base),
    _baseDirty  (false),
    _end        (Mat4f::Identity()),
    _endDirty   (false)
{
}

int64_t Chain::addJoint(const Joint &joint)
{
    _joints.push_back(joint);
    _endDirty = true;
    return _joints.size()-1;
}

void Chain::setBase(const Mat4f& base)
{
    _bm = base;
    _baseDirty = true;
    _endDirty = true;
}

void Chain::setJointAngle(int64_t id, float angle)
{
    if (id < 0 || id >= _joints.size()) {
        fprintf(stderr, "ERROR: Invalid joint id (%d)\n", (int)id);
        return;
    }

    _joints[id].setAngle(angle);
    _endDirty = true;
}

const Mat4f& Chain::getEnd()
{
    if (_endDirty)
        update();

    return _end;
}

void Chain::update()
{
    if (_joints.size() == 0)
        return;

    if (_baseDirty) {
        _joints[0].setBase(_bm);
        _baseDirty = false;
    }

    // calculate forward kinematics
    for (auto i=1; i<_joints.size(); ++i) {
        _joints[i].setBase(_joints[i-1].getEnd());
    }

    _end = _joints.back().getEnd();
    _endDirty = false;
}
