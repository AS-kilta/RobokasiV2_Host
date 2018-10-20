//
// Created by lehdari on 20.10.2018.
//

#ifndef ROBOKASIV2_HOST_GUI_GLTYPES_HPP
#define ROBOKASIV2_HOST_GUI_GLTYPES_HPP


#include <Eigen/Dense>
#include <GL/gl3w.h>


namespace gui {

    using Vec2GLf = Eigen::Matrix<GLfloat, 2, 1, Eigen::DontAlign>;
    using Vec3GLf = Eigen::Matrix<GLfloat, 3, 1, Eigen::DontAlign>;
    using Vec4GLf = Eigen::Matrix<GLfloat, 4, 1, Eigen::DontAlign>;

    using Mat2GLf = Eigen::Matrix<GLfloat, 2, 2, Eigen::DontAlign>;
    using Mat3GLf = Eigen::Matrix<GLfloat, 3, 3, Eigen::DontAlign>;
    using Mat4GLf = Eigen::Matrix<GLfloat, 4, 4, Eigen::DontAlign>;

} // namespace gui


#endif //ROBOKASIV2_HOST_GUI_GLTYPES_HPP
