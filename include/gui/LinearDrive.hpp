#ifndef ROBOKASIV2_HOST_GUI_LINEARDRIVE_HPP
#define ROBOKASIV2_HOST_GUI_LINEARDRIVE_HPP

#include "kinematics/LinearDrive.hpp"

namespace gui {
    struct LinearDrive : public kin::LinearDrive {
        LinearDrive(std::string name, size_t endPoseIdx);
        LinearDrive(const nlohmann::json& json);
        void edit(void);
    };
}

#endif
