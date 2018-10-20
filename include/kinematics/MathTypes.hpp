//
// Created by Lehdari on 25.6.2018.
//

#ifndef ROBOKASIV2_HOST_KINEMATICS_MATHTYPES_HPP
#define ROBOKASIV2_HOST_KINEMATICS_MATHTYPES_HPP


#include <Eigen/Dense>
#include <cstdint>


#define PI 3.14159265359


namespace kin {

    using Vec2f = Eigen::Vector2f;
    using Vec3f = Eigen::Vector3f;
    using Vec4f = Eigen::Vector4f;
    using Mat2f = Eigen::Matrix2f;
    using Mat3f = Eigen::Matrix3f;
    using Mat4f = Eigen::Matrix4f;

    using Vec2d = Eigen::Vector2d;
    using Vec3d = Eigen::Vector3d;
    using Vec4d = Eigen::Vector4d;
    using Mat2d = Eigen::Matrix2d;
    using Mat3d = Eigen::Matrix3d;
    using Mat4d = Eigen::Matrix4d;

} // namespace kin


#endif // ROBOKASIV2_HOST_KINEMATICS_MATHTYPES_HPP
