#include "kinematics/Program.hpp"

#include <json.hpp>

using json = nlohmann::json;
using namespace kin;

Puma560StepFrame::Puma560StepFrame(const Puma560& puma, int dt) :
    Puma560(puma),
    dt(dt)
{
}

ProgramStep::ProgramStep(std::string name, size_t endPoseIdx) :
    name(name),
    endPoseIdx(endPoseIdx)
{
}

void kin::to_json(json& j, const ProgramPose& pose)
{
    j = json{{"name", pose.name}, {"pose", pose.pose}};
}

void kin::to_json(json& j, const std::unique_ptr<ProgramStep>& step)
{
    step->toJson(j);
    j["name"] = step->name;
    j["typeName"] = step->getTypeName();
    j["endPoseIdx"] = step->endPoseIdx;
}
