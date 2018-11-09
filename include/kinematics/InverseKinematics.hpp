//
// Created by lehdari on 4.11.2018.
//

#ifndef ROBOKASIV2_HOST_INVERSEKINEMATICS_HPP
#define ROBOKASIV2_HOST_INVERSEKINEMATICS_HPP


#include "Chain.hpp"


namespace kin {

    /// Calculate inverse kinematics
    /// chain: previous pose
    /// pos: target position
    /// angles: target angles
    /// return: new pose
    Chain inverseKinematics(Chain chain, const Vec3f& pos, const Vec3f& angles);

} // namespace kin


#endif //ROBOKASIV2_HOST_INVERSEKINEMATICS_HPP
