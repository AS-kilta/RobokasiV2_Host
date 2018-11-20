#include "gui/PauseProgram.hpp"

#include <algorithm>

using namespace gui;

PauseProgram::PauseProgram(std::string name, size_t endPoseIdx) :
    ProgramStep(name, endPoseIdx, true)
{
}

PauseProgram::PauseProgram(nlohmann::json json) :
    ProgramStep(json, true)
{
}

std::vector<kin::Puma560StepFrame> PauseProgram::generate(const kin::Puma560& poseA,
                                                          const kin::Puma560& poseB)
{
    std::vector<kin::Puma560StepFrame> frames;
    frames.push_back({poseA, 1});

    return frames;
}

std::string PauseProgram::getTypeName()
{
    return typeName;
}

void PauseProgram::toJson(nlohmann::json& j)
{
}
