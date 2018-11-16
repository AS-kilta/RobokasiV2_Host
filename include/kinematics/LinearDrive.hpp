#ifndef ROBOKASIV2_HOST_KINEMATICS_LINEARDRIVE_HPP
#define ROBOKASIV2_HOST_KINEMATICS_LINEARDRIVE_HPP

#include "kinematics/Program.hpp"

namespace kin {
    struct LinearDrive : ProgramStep {
        LinearDrive(std::string name);
        std::vector<Puma560StepFrame> generate(const Puma560& poseA,
                                               const Puma560& poseB);
        /*
         * The linear drive program step can either generate multiple steps with
         * constant dt between each frame or it can create a single frame with
         * a large dt equaling the total transition time. The latter approach
         * relies on the linear interpolation of frames further down the
         * pipeline. The single frame mode should be used as the first step in
         * a program to slowly drive robot to the starting position.
         */
        bool multiFrame = false;
        /* Multi frame mode variables */
        float vel = 10.0f; /* deg / s */
        int dt = 100; /* ms */
        /* Single frame mode variables */
        int transitionTime = 5000; /* ms */
    };
}

#endif
