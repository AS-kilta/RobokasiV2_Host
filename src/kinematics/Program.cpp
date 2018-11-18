#include "kinematics/Program.hpp"

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
