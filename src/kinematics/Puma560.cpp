//
// Created by lehdari on 13.10.2018.
//

#include <kinematics/Puma560.hpp>

#include "Puma560.hpp"


using namespace kin;


Puma560::Puma560()
{
    _chain.addJoint(Joint(DHMatrix(0.0f, 0.0f, 0.0f, PI*0.5f)));
    _chain.addJoint(Joint(DHMatrix(0.0f, 0.0f, 0.4318f, 0.0f)));
    _chain.addJoint(Joint(DHMatrix(0.15f, 0.0f, 0.0203f, -PI*0.5f)));
    _chain.addJoint(Joint(DHMatrix(0.4318f, 0.0f, 0.0f, PI*0.5f)));
    _chain.addJoint(Joint(DHMatrix(0.0f, 0.0f, 0.0f, -PI*0.5f)));
    _chain.addJoint(Joint(DHMatrix(0.0f, 0.0f, 0.0f, 0.0f)));
}

void Puma560::setBase(const Mat4f& base)
{
    _chain.setBase(base);
}

bool Puma560::setJointAngle(int64_t id, float angle)
{
    // TODO add angle boundary checks here

    _chain.setJointAngle(id, angle);
    return true;
}

float Puma560::getJointAngle(int64_t id) const
{
    return _chain.getJointAngle(id);
}

const Chain& Puma560::getChain()
{
    _chain.update();
    return _chain;
}

const Mat4f& Puma560::getEnd()
{
    return _chain.getEnd();
}
