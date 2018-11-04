//
// Created by lehdari on 13.10.2018.
//

#ifndef ROBOKASIV2_HOST_KINEMATICS_JOINT_HPP
#define ROBOKASIV2_HOST_KINEMATICS_JOINT_HPP


#include "DHMatrix.hpp"


namespace kin {

    class Joint {
    public:
        Joint(const DHMatrix& dh,
            const Mat4f& base = Mat4f::Identity());

        /// Set base matrix
        void setBase(const Mat4f& base);

        /// Set angle around Z axis
        void setAngle(float angle);

        float getAngle() const;

        /// Get DH parameter matrix
        const DHMatrix& getDHMatrix(void) const;

        /// Get joint end
        const Mat4f& getEnd();

    private:
        DHMatrix    _dh; // DH parameter matrix

        Mat4f       _bm; // base matrix (previous joint end)

        Mat4f       _end; // end transformation matrix
        bool        _dirty;
    };

} // namespace kin


#endif //ROBOKASIV2_HOST_KINEMATICS_JOINT_HPP
