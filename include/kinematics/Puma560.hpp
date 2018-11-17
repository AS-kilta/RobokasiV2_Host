//
// Created by lehdari on 13.10.2018.
//

#ifndef ROBOKASIV2_HOST_KINEMATICS_PUMA560_HPP
#define ROBOKASIV2_HOST_KINEMATICS_PUMA560_HPP


#include "Chain.hpp"

#include <json.hpp>


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

        float getJointAngle(int64_t id) const;

        /// Get kinematic chain
        const Chain& getChain();

        /// Get end effector transformation
        const Mat4f& getEnd();

        bool gripper = 0;
    protected:
        Chain   _chain; // kinematic chain
    };

    void to_json(nlohmann::json& j, const Puma560& puma);

} // namespace kin


#endif //ROBOKASIV2_HOST_KINEMATICS_PUMA560_HPP
