//
// Created by lehdari on 13.10.2018.
//

#ifndef ROBOKASIV2_HOST_KINEMATICS_CHAIN_HPP
#define ROBOKASIV2_HOST_KINEMATICS_CHAIN_HPP


#include "Joint.hpp"
#include <vector>


namespace kin {

    class Chain {
    public:
        Chain(const Mat4f& base = Mat4f::Identity());

        /// Add joint
        /// return: id of the joint
        int64_t addJoint(const Joint& joint);

        /// Set base transformation
        void setBase(const Mat4f& base);

        /// Set joint angle
        void setJointAngle(int64_t id, float angle);

        /// Get end effector position of the kinematic chain
        const Mat4f& getEnd();
        /// Remember to call update() before using const version
        const Mat4f& getEnd() const;

        /// Update forward kinematics
        void update();

    private:
        Mat4f               _bm; // base transformation matrix
        bool                _baseDirty;
        std::vector<Joint>  _joints;
        Mat4f               _end; // end effector transformation
        bool                _endDirty;
    };

} // namespace kin


#endif // ROBOKASIV2_HOST_KINEMATICS_CHAIN_HPP
