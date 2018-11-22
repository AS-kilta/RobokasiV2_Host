#ifndef ROBOKASIV2_HOST_KINEMATICS_TRAPEZOIDDRIVE_HPP
#define ROBOKASIV2_HOST_KINEMATICS_TRAPEZOIDDRIVE_HPP

#include "kinematics/Program.hpp"

namespace kin {
    struct TrapezoidDrive : ProgramStep {
        TrapezoidDrive(std::string name, size_t endPoseIdx);
        TrapezoidDrive(nlohmann::json j);
        std::vector<Puma560StepFrame> generate(const Puma560& poseA,
                                               const Puma560& poseB);
        virtual std::string getTypeName();
        void toJson(nlohmann::json& j);
        static constexpr char typeName[] = "TrapezoidDrive";

        float accel = 0.5f; /* deg / s ^ 2 */
        float vel = 10.0f; /* deg / s */
        int dt = 50; /* ms */
    };
}

#endif
