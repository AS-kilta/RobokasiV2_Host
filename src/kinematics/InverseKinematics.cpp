//
// Created by lehdari on 4.11.2018.
//

#include "InverseKinematics.hpp"


using namespace kin;


namespace {

    void normalizeAngles(Vec3f& a)
    {
        if (a(0) < -PI*0.5f) a(0) += PI;
        if (a(1) < -PI*0.5f) a(1) += PI;
        if (a(2) < -PI*0.5f) a(2) += PI;
        if (a(0) > PI*0.5f) a(0) -= PI;
        if (a(1) > PI*0.5f) a(1) -= PI;
        if (a(2) > PI*0.5f) a(2) -= PI;
    }

}


Chain kin::inverseKinematics(Chain chain, const Vec3f& pos, Vec3f angles)
{
    const float d = 0.00174532f; // 0.1 degrees
    const float i2d = 0.5f/d;

    Mat3f destRot =
        Eigen::AngleAxisf(angles(0), Vec3f::UnitY()).toRotationMatrix() *
        Eigen::AngleAxisf(angles(1), Vec3f::UnitX()).toRotationMatrix() *
        Eigen::AngleAxisf(angles(2), Vec3f::UnitZ()).toRotationMatrix();

    angles = destRot.eulerAngles(1, 0, 2);

    auto nJoints = chain.getJointCount();
    Eigen::Matrix<float, 6, Eigen::Dynamic> jacob(6, nJoints);
    Eigen::Matrix<float, Eigen::Dynamic, 1> jointAngles(nJoints);

    Chain newChain = chain;

    Vec4f orig(0.0f, 0.0f, 0.0f, 1.0f);
    Vec3f p((chain.getEnd() * orig).block<3, 1>(0, 0));
    Vec3f a(chain.getEnd().block<3, 3>(0, 0).eulerAngles(1, 0, 2));

    float dp = (pos - p).norm();
    float da = (angles - a).norm();

    int64_t itn = 0;
    while ((dp > 1.0f || da > 0.1f) && itn < 10000) {
        //printf("cur: %12.5f, %12.5f, %12.5f\n", a(0), a(1), a(2));
        //printf("des: %12.5f, %12.5f, %12.5f\n", angles(0), angles(1), angles(2));

        for (uint64_t i = 0u; i < nJoints; ++i) {
            jointAngles(i) = chain.getJointAngle(i);
            Mat4f tEnd1 = chain.getEnd();
            Mat4f tEnd1i = tEnd1.inverse();
            Vec4f pEnd(tEnd1 * orig);

            float theta = chain.getJointAngle(i);
            chain.setJointAngle(i, theta - d);
            Mat4f tEnd2 = chain.getEnd();
            Mat4f tEnd2r = tEnd1i * tEnd2;

            chain.setJointAngle(i, theta + d);
            Mat4f tEnd3 = chain.getEnd();
            Mat4f tEnd3r = tEnd1i * tEnd3;

            Vec4f pd = tEnd3 * orig - tEnd2 * orig;

            jacob(0, i) = pd(0) * i2d;
            jacob(1, i) = pd(1) * i2d;
            jacob(2, i) = pd(2) * i2d;

            Vec3f tEnd2Angles = tEnd2.block<3, 3>(0, 0).eulerAngles(1, 0, 2);
            Vec3f tEnd3Angles = tEnd3.block<3, 3>(0, 0).eulerAngles(1, 0, 2);

            Vec3f diff = tEnd3Angles - tEnd2Angles;

//            printf("te1: %12.5f, %12.5f, %12.5f\n", tEnd2Angles(0), tEnd2Angles(1), tEnd2Angles(2));
//            printf("te2: %12.5f, %12.5f, %12.5f\n", tEnd3Angles(0), tEnd3Angles(1), tEnd3Angles(2));
//            printf("dif: %12.5f, %12.5f, %12.5f\n", diff(0), diff(1), diff(2));
/*
            Vec3f dAngleCos(
                tEnd3.block<1, 3>(0, 0).dot(tEnd2.block<1, 3>(0, 0)),
                tEnd3.block<1, 3>(1, 0).dot(tEnd2.block<1, 3>(1, 0)),
                tEnd3.block<1, 3>(2, 0).dot(tEnd2.block<1, 3>(2, 0)));
            printf("dac: %12.5f, %12.5f, %12.5f\n", dAngleCos(0), dAngleCos(1), dAngleCos(2));

            dAngleCos(0) = std::clamp(dAngleCos(0), -0.999999f, 0.999999f);
            dAngleCos(1) = std::clamp(dAngleCos(1), -0.999999f, 0.999999f);
            dAngleCos(2) = std::clamp(dAngleCos(2), -0.999999f, 0.999999f);
            diff(0) = acos(dAngleCos(0));
            diff(1) = acos(dAngleCos(1));
            diff(2) = acos(dAngleCos(2));
            printf("dif: %12.5f, %12.5f, %12.5f\n", diff(0), diff(1), diff(2));
*/
            jacob(3, i) = diff(0) * i2d;
            jacob(4, i) = diff(1) * i2d;
            jacob(5, i) = diff(2) * i2d;

            chain = newChain;
        }

        // pseudoinverse
        Eigen::Matrix<float, 6, Eigen::Dynamic> jacobPI =
            jacob.completeOrthogonalDecomposition().pseudoInverse();

#if 0
        for (int j = 0; j < 6; ++j) {
            for (uint64_t i = 0u; i < nJoints; ++i) {
                printf("%10.5f ", jacob(j, i));
            }
            printf("\n");
        }
        printf("\n");
        for (int j = 0; j < 6; ++j) {
            for (uint64_t i = 0u; i < nJoints; ++i) {
                printf("%10.5f ", jacobPI(j, i));
            }
            printf("\n");
        }
        printf("\n");
#endif

        Eigen::Matrix<float, 6, 1> deltap;
        deltap << (pos - p).normalized()*0.1f, (angles - a).normalized()*0.001f;
        Eigen::Matrix<float, 6, 1> angleAdd = jacobPI * deltap;
        jointAngles += angleAdd;

        for (uint64_t i = 0u; i < nJoints; ++i) {
            newChain.setJointAngle(i, jointAngles(i));
            //printf("%0.5f\t%0.5f\t%0.5f\n ", deltap(i), angleAdd(i), jointAngles(i));
        }
        //printf("\n");

        chain = newChain;

        p = (chain.getEnd() * orig).block<3, 1>(0, 0);
        a = chain.getEnd().block<3, 3>(0, 0).eulerAngles(1, 0, 2);

        dp = (pos - p).norm();
        da = (angles - a).norm();

        //printf("it: %ld\t", itn++);
        //sprintf("dp: %12.5f\t", dp);
        //printf("da: %12.5f\n", da);
    }

    return newChain;
}
