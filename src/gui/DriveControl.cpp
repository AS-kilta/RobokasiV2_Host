#include "gui/DriveControl.hpp"

#include <imgui.h>


using namespace gui;


DriveControl::DriveControl(hwio::SerialProto& serialProto) :
    _serialProto(serialProto),
    _initialControlsSet(false),
    _contiguousMode(false)
{
}

void DriveControl::render(void)
{
    hwio::State state;

    if (!_serialProto.isConnected())
        return;

    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiSetCond_Once);
    ImGui::SetNextWindowSize(ImVec2(400, 500), ImGuiSetCond_Once);

    ImGui::Begin("Drive Control");

    state = _serialProto.getState();

    ImGui::Text("Joint angles");
    for (const float angle : state.angles)
        ImGui::ProgressBar(angle);

    ImGui::Text("Safemode: %d, Brake: %d, Gripper: %d\n", state.safemode,
                state.brake, state.gripper);

    ImGui::Separator();

    if (ImGui::Button("Init") || !_initialControlsSet) {
        _command = hwio::Command(state, 16);
        _initialControlsSet = true;
    }

    bool edited = 0;
    /* TODO Move and enforce joint limits elsewhere */
    edited |= ImGui::SliderFloat("j1", &_command.angles[0], 0.270f, 0.830f);
    edited |= ImGui::SliderFloat("j2", &_command.angles[1], 0.155f, 0.99f);
    edited |= ImGui::SliderFloat("j3", &_command.angles[2], 0.357f, 0.761f);
    edited |= ImGui::SliderFloat("j4", &_command.angles[3], 0.280f, 0.960);
    edited |= ImGui::SliderFloat("j5", &_command.angles[4], 0.355f, 0.752);
    edited |= ImGui::SliderFloat("j6", &_command.angles[5], 0.0f, 1.0f);

    ImGui::Checkbox("Safemode", &_command.safemode);
    ImGui::SameLine();
    ImGui::Checkbox("Brake", &_command.brake);
    ImGui::SameLine();
    ImGui::Checkbox("Gripper", &_command.gripper);

    ImGui::Separator();

    ImGui::SliderInt("dt", &_command.dt, 16, 1000);

    if (ImGui::Button("Go") || (edited && _contiguousMode))
        _serialProto.sendCommand(_command);
    ImGui::SameLine();
    ImGui::Checkbox("Contiguous mode", &_contiguousMode);

    ImGui::Separator();
    ImGui::Text("Controller command buffer utilization");
    ImGui::ProgressBar(_serialProto.getBufStatus());

    ImGui::End();
}
