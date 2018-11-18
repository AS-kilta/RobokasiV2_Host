#include "kinematics/LinearDrive.hpp"

#include <algorithm>

using namespace kin;

LinearDrive::LinearDrive(std::string name, size_t endPoseIdx) :
    ProgramStep(name, endPoseIdx)
{
}

LinearDrive::LinearDrive(nlohmann::json json) :
    ProgramStep(json)
{
    multiFrame = json["multiFrame"].get<bool>();
    if (multiFrame) {
        vel = json["vel"].get<float>();
        dt = json["dt"].get<int>();
    } else {
        transitionTime = json["transitionTime"].get<int>();
    }
}

std::vector<Puma560StepFrame> LinearDrive::generate(const Puma560& poseA,
                                                    const Puma560& poseB)
{
    std::vector<Puma560StepFrame> frames;

    if (multiFrame) {
        std::array<float, 6> deltas;
        std::array<float, 6> absDeltas;

        for (size_t i = 0; i < 6; ++i) {
            deltas[i] = poseB.getJointAngle(i) - poseA.getJointAngle(i);
            absDeltas[i] = fabs(deltas[i]);
        }

        float maxDelta = *std::max_element(begin(absDeltas), end(absDeltas));

        if (maxDelta == 0.0f)
            return frames;

        float velRadPerMs = vel * (PI / 180) * 0.001;
        int t = maxDelta / velRadPerMs;
        size_t numFrames = t / dt;

        for (size_t i = 0; i < 6; ++i)
            deltas[i] *= (float)dt / t;

        for (size_t i = 0; i < numFrames; ++i) {
            Puma560StepFrame frame;
            for (size_t j = 0; j < 6; ++j)
                frame.setJointAngle(j, poseA.getJointAngle(j) + i * deltas[j]);
            frame.dt = dt;
            frames.push_back(frame);
        }
    } else {
        frames.emplace_back(poseB, transitionTime);
    }

    return frames;
}

std::string LinearDrive::getTypeName()
{
    return typeName;
}

void LinearDrive::toJson(nlohmann::json& j)
{
    j["multiFrame"] = multiFrame;

    if (multiFrame) {
        j["vel"] = vel;
        j["dt"] = dt;
    } else {
        j["transitionTime"] = transitionTime;
    }
}
