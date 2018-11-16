#include "gui/LinearDrive.hpp"

#include <imgui.h>

using namespace gui;

LinearDrive::LinearDrive(std::string name) :
    kin::LinearDrive(name)
{
}

void LinearDrive::edit(void)
{
    ImGui::Checkbox("Multi frame mode", &multiFrame);
    if (multiFrame) {
        ImGui::InputFloat("vel (deg / s)", &vel);
        ImGui::InputInt("dt (ms)", &dt);
    } else {
        ImGui::InputInt("Ttrans (ms)", &transitionTime);
    }
}

