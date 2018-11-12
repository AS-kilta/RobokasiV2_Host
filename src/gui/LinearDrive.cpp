#include "gui/LinearDrive.hpp"

#include <imgui.h>

using namespace gui;

LinearDrive::LinearDrive(std::string name) :
    kin::LinearDrive(name)
{
}

void LinearDrive::edit(void)
{
    ImGui::InputFloat("vel (deg / s)", &vel);
    ImGui::InputInt("dt (ms)", &dt);
}

