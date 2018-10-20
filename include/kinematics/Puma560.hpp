//
// Created by lehdari on 13.10.2018.
//

#ifndef ROBOKASIV2_HOST_KINEMATICS_PUMA560_HPP
#define ROBOKASIV2_HOST_KINEMATICS_PUMA560_HPP


#include "Chain.hpp"


namespace kin {

    class Puma560 {
    public:
        Puma560();

        /// Set base transformation
        void setBase(const Mat4f& base);

        /// Set joint angle
        /// id: joint id
        /// angle: joint angle
        /// return: boolean whether position is valid
        bool setJointAngle(int64_t id, float angle);

        /// Get kinematic chain
        const Chain& getChain();

        /// Get end effector transformation
        const Mat4f& getEnd();

    protected:
        Chain   _chain; // kinematic chain
    };

} // namespace kin


#endif //ROBOKASIV2_HOST_KINEMATICS_PUMA560_HPP
