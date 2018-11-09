//
// Created by lehdari on 4.11.2018.
//

#include "InverseKinematics.hpp"


using namespace kin;


Chain kin::inverseKinematics(Chain chain, const Vec3f& pos, const Vec3f& angles)
{
    const float d = 0.00174532f; // 0.1 degrees
    const float i2d = 0.5f/d;

    auto nJoints = chain.getJointCount();
    Eigen::Matrix<float, 6, Eigen::Dynamic> jacob(6, nJoints);
    Eigen::Matrix<float, Eigen::Dynamic, 1> jointAngles(nJoints);

    Chain newChain = chain;

    Vec4f orig(0.0f, 0.0f, 0.0f, 1.0f);

    for (int k=0; k<100; ++k) {
        Vec3f p((chain.getEnd() * orig).block<3, 1>(0, 0));
        Vec3f a(chain.getEnd().block<3, 3>(0, 0).eulerAngles(0, 1, 2));

        for (uint64_t i = 0u; i < nJoints; ++i) {
            jointAngles(i) = chain.getJointAngle(i);
            Mat4f tEnd1 = chain.getEnd();
            Vec4f pEnd(tEnd1 * orig);

            float theta = chain.getJointAngle(i);
            chain.setJointAngle(i, theta - d);
            Mat4f tEnd2 = chain.getEnd();

            chain.setJointAngle(i, theta + d);
            Mat4f tEnd3 = chain.getEnd();

            Vec4f pd = tEnd3 * orig - tEnd2 * orig;

            jacob(0, i) = pd(0) * i2d;
            jacob(1, i) = pd(1) * i2d;
            jacob(2, i) = pd(2) * i2d;

            Vec3f tEnd2Angles = tEnd2.block<3, 3>(0, 0).eulerAngles(0, 1, 2);
            Vec3f tEnd3Angles = tEnd3.block<3, 3>(0, 0).eulerAngles(0, 1, 2);

            jacob(3, i) = (tEnd3Angles(0) - tEnd2Angles(0)) * i2d;
            jacob(4, i) = (tEnd3Angles(1) - tEnd2Angles(1)) * i2d;
            jacob(5, i) = (tEnd3Angles(2) - tEnd2Angles(2)) * i2d;

            chain = newChain;
        }

        //Eigen::Matrix<float, 6, 1> de = ;

        // pseudoinverse
        Eigen::Matrix<float, 6, Eigen::Dynamic> jacobPI =
            (jacob * jacob.transpose()).inverse() * jacob.transpose();
/*
        for (int j = 0; j < 6; ++j) {
            for (uint64_t i = 0u; i < nJoints; ++i) {
                printf("%10.5f ", jacob(j, i));
            }
            printf("\n");
        }
        printf("\n");
        for (int j = 0; j < 6; ++j) {
            for (uint64_t i = 0u; i < nJoints; ++i) {
                printf("%10.5f ", jacobPI(j, i));f
            }
            printf("\n");
        }
        printf("\n");
*/
        Eigen::Matrix<float, 6, 1> deltap;
        deltap << (pos - p).normalized()*0.0001f, (angles - a).normalized()*0.0001f;
        Eigen::Matrix<float, 6, 1> angleAdd = jacob.transpose() * deltap;
        jointAngles += angleAdd;

        for (uint64_t i = 0u; i < nJoints; ++i) {
            newChain.setJointAngle(i, jointAngles(i));
            //printf("%0.5f\t%0.5f\t%0.5f\n ", deltap(i), angleAdd(i), jointAngles(i));
        }
        //printf("\n");

        //printf("dp: %0.5f\n", (pos - p).norm());
        //printf("da: %0.5f\n", (angles - a).norm());

        chain = newChain;
    }

    return newChain;
}
