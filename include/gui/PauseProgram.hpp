#ifndef ROBOKASIV2_HOST_GUI_OPERATORWAIT_HPP
#define ROBOKASIV2_HOST_GUI_OPERATORWAIT_HPP

#include "kinematics/Program.hpp"

namespace gui {
    struct PauseProgram : kin::ProgramStep {
        PauseProgram(const std::string name, size_t endPoseIdx);
        PauseProgram(nlohmann::json j);
        std::vector<kin::Puma560StepFrame> generate(const kin::Puma560& poseA,
                                                    const kin::Puma560& poseB);
        virtual std::string getTypeName();
        void toJson(nlohmann::json& j);
        static constexpr char typeName[] = "PauseProgram";
    };
}

#endif
