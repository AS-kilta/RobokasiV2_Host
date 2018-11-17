#ifndef ROBOKASIV2_HOST_GUI_STEPTYPES_HPP
#define ROBOKASIV2_HOST_GUI_STEPTYPES_HPP

#include "kinematics/LinearDrive.hpp"

namespace gui {
    enum StepTypes {
        LinearDriveStep,
    };

    const char * const stepTypeNames[] = {
        [LinearDriveStep] = kin::LinearDrive::typeName,
    };

    constexpr int numStepTypeNames = sizeof(stepTypeNames) / sizeof(stepTypeNames[0]);

    int getStepTypeNameIdx(const char* typeName);
}

#endif
