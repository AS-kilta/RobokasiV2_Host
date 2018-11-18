#ifndef ROBOKASIV2_KINEMATICS_PROGRAM_HPP
#define ROBOKASIV2_KINEMATICS_PROGRAM_HPP

#include "kinematics/Puma560.hpp"

#include <json.hpp>

#include <memory>
#include <vector>
#include <string>

namespace kin {

    struct Puma560StepFrame : public kin::Puma560 {
        Puma560StepFrame() = default;
        Puma560StepFrame(const Puma560& puma, int dt);
        int dt;
    };

    struct ProgramStep {
        ProgramStep(std::string name, size_t endPoseIdx);
        virtual ~ProgramStep() {}
        virtual std::vector<Puma560StepFrame> generate(const Puma560& poseA,
                                                       const Puma560& poseB) = 0;
        virtual void edit(void) { }
        virtual void toJson(nlohmann::json& j) = 0;
        virtual std::string getTypeName() = 0;
        std::string name;
        size_t endPoseIdx;
    };

    struct ProgramPose {
        Puma560 pose;
        std::string name;
    };

    void to_json(nlohmann::json& j, const ProgramPose& pose);

    void to_json(nlohmann::json& j, const std::unique_ptr<ProgramStep>& step);

    struct Program {
        template <typename StepType, typename... Args>
        void addStep(Args&&... args)
        {
            steps.emplace_back(std::make_unique<StepType>(std::forward<Args>(args)...));
        }
        template <typename StepType, typename... Args>
        void addStep(size_t i, Args&&... args)
        {
            if (!steps.empty() && i < steps.size())
                steps.insert(steps.begin() + i, std::make_unique<StepType>(std::forward<Args>(args)...));
            else
                steps.emplace_back(std::make_unique<StepType>(std::forward<Args>(args)...));
        }
        std::vector<std::unique_ptr<ProgramStep>> steps;
        std::vector<ProgramPose> poses;
    };

}

#endif
