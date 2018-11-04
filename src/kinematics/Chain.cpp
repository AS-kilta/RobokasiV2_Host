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

const Mat4f& Chain::getBase() const
{
    return _bm;
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

float Chain::getJointAngle(int64_t id) const
{
    if (id < 0 || id >= _joints.size()) {
        fprintf(stderr, "ERROR: Invalid joint id (%d)\n", (int)id);
        return 0.0f;
    }

    return _joints[id].getAngle();
}

uint64_t Chain::getJointCount() const
{
    return _joints.size();
}

const Mat4f& Chain::getJointEnd(int64_t id)
{
    if (id < 0 || id >= _joints.size()) {
        fprintf(stderr, "ERROR: Invalid joint id (%d)\n", (int)id);
        return Mat4f::Identity();
    }

    update();
    return _joints[id].getEnd();
}

const Mat4f& Chain::getJointJointMatrix(int64_t id)
{
    if (id < 0 || id >= _joints.size()) {
        fprintf(stderr, "ERROR: Invalid joint id (%d)\n", (int)id);
        return Mat4f::Identity();
    }

    update();
    return _joints[id]._dh.getJointMatrix();
}

const Mat4f& Chain::getEnd()
{
    update();
    return _end;
}

const Mat4f& Chain::getEnd() const
{
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

    if (_endDirty) {
        // calculate forward kinematics
        for (auto i = 1; i < _joints.size(); ++i) {
            _joints[i].setBase(_joints[i - 1].getEnd());
        }

        _end = _joints.back().getEnd();
        _endDirty = false;
    }
}
