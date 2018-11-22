#ifndef ROBOKASIV2_HOST_GUI_TRAPEZOIDDRIVE_HPP
#define ROBOKASIV2_HOST_GUI_TRAPEZOIDDRIVE_HPP

#include "kinematics/TrapezoidDrive.hpp"

namespace gui {
    struct TrapezoidDrive : public kin::TrapezoidDrive {
        TrapezoidDrive(std::string name, size_t endPoseIdx);
        TrapezoidDrive(const nlohmann::json& json);
        void edit(void);
    };
}

#endif
