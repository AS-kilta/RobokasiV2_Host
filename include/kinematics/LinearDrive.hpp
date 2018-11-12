#ifndef ROBOKASIV2_HOST_KINEMATICS_LINEARDRIVE_HPP
#define ROBOKASIV2_HOST_KINEMATICS_LINEARDRIVE_HPP

#include "kinematics/Program.hpp"

namespace kin {
    struct LinearDrive : ProgramStep {
        LinearDrive(std::string name);
        std::vector<Puma560StepFrame> generate(const Puma560& poseA,
                                               const Puma560& poseB);
        float vel = 10.0f; /* deg / s */
        int dt = 100; /* ms */
    };
}

#endif
