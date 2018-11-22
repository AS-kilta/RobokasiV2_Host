#include "gui/TrapezoidDrive.hpp"

#include <imgui.h>

using namespace gui;

TrapezoidDrive::TrapezoidDrive(std::string name, size_t endPoseIdx) :
    kin::TrapezoidDrive(name, endPoseIdx)
{
}

TrapezoidDrive::TrapezoidDrive(const nlohmann::json& json) :
    kin::TrapezoidDrive(json)
{
}

void TrapezoidDrive::edit(void)
{
    ImGui::InputFloat("accel (deg / s^2)", &accel);
    ImGui::InputFloat("vel (deg / s)", &vel);
    ImGui::InputInt("dt (ms)", &dt);
}
