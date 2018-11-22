#include "kinematics/TrapezoidDrive.hpp"

#include <algorithm>

using namespace kin;

TrapezoidDrive::TrapezoidDrive(std::string name, size_t endPoseIdx) :
    ProgramStep(name, endPoseIdx)
{
}

TrapezoidDrive::TrapezoidDrive(nlohmann::json json) :
    ProgramStep(json)
{
    accel = json["accel"].get<float>();
    vel = json["vel"].get<float>();
    dt = json["dt"].get<int>();
}

static std::vector<float> genTrapezoid(float x0, float x1, float a, float v_max,
                                       float dt)
{
    std::vector<float> p;
    float ramp_len;
    float x = x0;
    float v = 0.0f;
    enum class Phase {
        Accel,
        Linear,
        Decel,
    } phase = Phase::Accel;

    v_max *= 0.1; // Dunnolol

    printf("%f, %f\n", x0, x1);
    while (x <= x1 && v >= 0) {
        fprintf(stderr, "p: %d, x: %f, v: %f\n", phase, x, v);
        switch (phase) {
        case Phase::Accel:
            v += a * dt;
            if (v >= v_max || x >= (x0 + x1) * 0.5f) {
                ramp_len = x;
                phase = Phase::Linear;
                printf("rl: %f\n", ramp_len);
            }
            break;
        case Phase::Linear:
            if (x1 - x <= ramp_len)
                phase = Phase::Decel;
            break;
        case Phase::Decel:
            v -= a * dt;
            break;
        }

        x += v * dt;

        p.push_back(x);
    }

    return p;
}

std::vector<Puma560StepFrame> TrapezoidDrive::generate(const Puma560& poseA,
                                                       const Puma560& poseB)
{
#if 1
    std::array<std::vector<float>, 6> angleFrames;
    size_t maxFrames = 0;

    for (size_t i = 0; i < 6; ++i) {
        if (poseA.getJointAngle(i) < poseB.getJointAngle(i)) {
            angleFrames[i] = genTrapezoid(poseA.getJointAngle(i),
                                          poseB.getJointAngle(i),
                                          accel, vel, dt / 1000.0f);
        } else {
            /* genTrapezoid can only handle cases where a > b */
            angleFrames[i] = genTrapezoid(poseB.getJointAngle(i),
                                          poseA.getJointAngle(i),
                                          accel, vel, dt / 1000.0f);
            std::reverse(begin(angleFrames[i]), end(angleFrames[i]));
        }
        if (angleFrames[i].size() > maxFrames)
            maxFrames = angleFrames[i].size();
    }

    std::vector<Puma560StepFrame> frames;
    for (size_t i = 0; i < maxFrames; ++i) {
        Puma560StepFrame frame;
        for (size_t j = 0; j < 6; ++j) {
            size_t idx = std::min(i, angleFrames[j].size() - 1);
            frame.setJointAngle(j, angleFrames[j][idx]);
        }
        frame.dt = dt;
        frames.push_back(frame);
    }

    return frames;
#else
    std::vector<Puma560StepFrame> frames;

    std::array<float, 6> deltas;
    //std::array<float, 6> deltavs;
    std::array<float, 6> angles;
    std::array<float, 6> absDeltas;

    for (size_t i = 0; i < 6; ++i) {
        angles[i] = poseA.getJointAngle(i);
        deltas[i] = poseB.getJointAngle(i) - angles[i];
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
        for (size_t j = 0; j < 6; ++j)
            angles[j] += deltas[j];

        Puma560StepFrame frame;
        for (size_t j = 0; j < 6; ++j)
            frame.setJointAngle(j, angles[j]);
        frame.dt = dt;
        frames.push_back(frame);
    }

    return frames;
#endif
}

std::string TrapezoidDrive::getTypeName()
{
    return typeName;
}

void TrapezoidDrive::toJson(nlohmann::json& j)
{
    j["accel"] = accel;
    j["vel"] = vel;
    j["dt"] = dt;
}
