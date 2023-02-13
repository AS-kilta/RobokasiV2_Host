//
// Created by lehdari on 4.11.2018.
//

#include "InverseKinematics.hpp"


using namespace kin;


namespace {

    /// angular error function for two rotation matrices
    float orientationError(const Mat3f& m1, const Mat3f& m2)
    {
        Vec3f a(
            m1.block<1, 3>(0, 0).dot(m2.block<1, 3>(0, 0)),
            m1.block<1, 3>(1, 0).dot(m2.block<1, 3>(1, 0)),
            m1.block<1, 3>(2, 0).dot(m2.block<1, 3>(2, 0)));
        Vec3f ac(std::acos(std::clamp(a(0), -0.999999f, 0.999999f)),
                std::acos(std::clamp(a(1), -0.999999f, 0.999999f)),
                std::acos(std::clamp(a(2), -0.999999f, 0.999999f)));
        return ac.squaredNorm();
    }

}


Chain kin::inverseKinematics(Chain chain, const Vec3f& pos, Vec3f angles)
{
    // constants for calculating jacobian matrix
    constexpr float d = 0.00174532f; // 0.1 degrees
    constexpr float i2d = 0.5f/d;

    // maximum number of iterations
    constexpr int64_t maxIters = 1000;

    // optimize pose until position / angles are within these thresholds
    constexpr float posErrorThreshold = 0.001f;
    constexpr float angleErrorThreshold = 0.001f;

    // destination rotation matrix
    Mat3f dest =
        Eigen::AngleAxisf(angles(0), Vec3f::UnitZ()).toRotationMatrix() *
        Eigen::AngleAxisf(angles(1), Vec3f::UnitY()).toRotationMatrix() *
        Eigen::AngleAxisf(angles(2), Vec3f::UnitZ()).toRotationMatrix();

    // number of joints in chain
    auto nJoints = chain.getJointCount();

    // some helper matrices and vectors
    Eigen::Matrix<float, 4, Eigen::Dynamic> jacob(4, nJoints);
    Eigen::Matrix<float, Eigen::Dynamic, 4> jacobPI(nJoints, 4);
    Eigen::Matrix<float, Eigen::Dynamic, 1> jointAngles(nJoints), angleAdd(nJoints);

    Chain newChain = chain;

    Vec4f orig(0.0f, 0.0f, 0.0f, 1.0f);

    // current position of the end effector
    Vec3f p((chain.getEnd() * orig).block<3, 1>(0, 0));

    // error metrics for position and orientation
    float dp = (pos - p).norm();
    float da = orientationError(chain.getEnd().block<3, 3>(0, 0), dest);

    // iterate until both errors are small enough or maximum no. of iterations reached
    for (int64_t itn = 0; (dp > posErrorThreshold || da > angleErrorThreshold) && itn < maxIters; ++itn) {
        // form jacobian matrix numerically
        for (uint64_t i = 0u; i < nJoints; ++i) {
            jointAngles(i) = chain.getJointAngle(i);
            Mat4f tEnd1 = chain.getEnd();
            Vec4f pEnd(tEnd1 * orig);
            float theta = chain.getJointAngle(i);

            // trapezoidal rule
            chain.setJointAngle(i, theta - d);
            Mat4f tEnd2 = chain.getEnd();

            chain.setJointAngle(i, theta + d);
            Mat4f tEnd3 = chain.getEnd();

            Vec4f pd = tEnd3 * orig - tEnd2 * orig;

            jacob(0, i) = pd(0) * i2d;
            jacob(1, i) = pd(1) * i2d;
            jacob(2, i) = pd(2) * i2d;

            jacob(3, i) = (orientationError(tEnd3.block<3, 3>(0, 0), dest) -
                orientationError(tEnd2.block<3, 3>(0, 0), dest)) * i2d;

            chain = newChain;
        }

        // pseudoinverse
        jacobPI = jacob.completeOrthogonalDecomposition().pseudoInverse();

        // update angles
        Eigen::Matrix<float, 4, 1> deltap;
        deltap << (pos - p)*0.1f,
            -orientationError(chain.getEnd().block<3, 3>(0, 0), dest)*0.1f;
        angleAdd = jacobPI * deltap;
        jointAngles += angleAdd;

        for (uint64_t i = 0u; i < nJoints; ++i)
            newChain.setJointAngle(i, jointAngles(i));

        // update chain an calculate new errors
        chain = newChain;
        p = (chain.getEnd() * orig).block<3, 1>(0, 0);

        dp = (pos - p).norm();
        da = orientationError(chain.getEnd().block<3, 3>(0, 0), dest);
    }

    return newChain;
}
