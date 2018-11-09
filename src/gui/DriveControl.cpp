#include "gui/DriveControl.hpp"
#include "kinematics/MathTypes.hpp"
#include "hwio/CommandQueue.hpp"

#include <imgui.h>


using namespace gui;


DriveControl::DriveControl(hwio::SerialProto& serialProto, hwio::CommandQueue& cq) :
    _serialProto(serialProto),
    _commandQueue(cq),
    _initialControlsSet(false),
    _contiguousMode(false)
{
}

void DriveControl::render(void)
{
    float deg_angles[6];
    hwio::State state;
    bool edited = 0;

    if (!_serialProto.isConnected())
        return;

    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiSetCond_Once);
    ImGui::SetNextWindowSize(ImVec2(400, 500), ImGuiSetCond_Once);

    ImGui::Begin("Drive Control");

    state = _serialProto.getState();

    ImGui::Text("Joint angles");

    for (int i = 0; i< 6; ++i)
        deg_angles[i] = state.angles[i] / PI * 180;

    ImGui::SliderFloat("j1", &deg_angles[0], -180.0f, 180.0f);
    ImGui::SliderFloat("j2", &deg_angles[1], -180.0f, 180.0f);
    ImGui::SliderFloat("j3", &deg_angles[2], -180.0f, 180.0f);
    ImGui::SliderFloat("j4", &deg_angles[3], -180.0f, 180.0f);
    ImGui::SliderFloat("j5", &deg_angles[4], -180.0f, 180.0f);
    ImGui::SliderFloat("j6", &deg_angles[5], -180.0f, 180.0f);


    ImGui::Text("Safemode: %d, Brake: %d, Gripper: %d\n", state.safemode,
                state.brake, state.gripper);

    ImGui::Separator();

    if (ImGui::Button("Init") || !_initialControlsSet) {
        _command = hwio::Command(state, 16);
        _initialControlsSet = true;
    }

    ImGui::SameLine();
    if (ImGui::Button("L-position")) {
        for (int i = 0; i < 6; ++i)
            _command.angles[i] = 0.0f;
        _command.dt = 5000;
        edited = 1;
    }

    ImGui::SameLine();
    if (ImGui::Button("Straight")) {
        _command.angles = { 0.0f, PI / 2.0f, -PI / 2.0f,
                            0.0f, 0.0f, 0.0f };
        _command.dt = 5000;
        edited = 1;
    }

    for (int i = 0; i< 6; ++i)
        deg_angles[i] = _command.angles[i] / PI * 180;

    /* TODO Move and enforce joint limits elsewhere */
    edited |= ImGui::SliderFloat("j1", &deg_angles[0], -180.0f, 180.0f);
    edited |= ImGui::SliderFloat("j2", &deg_angles[1], -180.0f, 180.0f);
    edited |= ImGui::SliderFloat("j3", &deg_angles[2], -180.0f, 180.0f);
    edited |= ImGui::SliderFloat("j4", &deg_angles[3], -180.0f, 180.0f);
    edited |= ImGui::SliderFloat("j5", &deg_angles[4], -180.0f, 180.0f);
    edited |= ImGui::SliderFloat("j6", &deg_angles[5], -180.0f, 180.0f);

    for (int i = 0; i< 6; ++i)
        _command.angles[i] = deg_angles[i] / 180 * PI;

    ImGui::Checkbox("Safemode", &_command.safemode);
    ImGui::SameLine();
    ImGui::Checkbox("Brake", &_command.brake);
    ImGui::SameLine();
    ImGui::Checkbox("Gripper", &_command.gripper);

    ImGui::Separator();

    ImGui::SliderInt("dt", &_command.dt, 16, 5000);

    if (ImGui::Button("Go") || (edited && _contiguousMode))
        _commandQueue.addCommand(_command);
    ImGui::SameLine();
    ImGui::Checkbox("Contiguous mode", &_contiguousMode);

    ImGui::Separator();
    ImGui::Text("Controller command buffer utilization");
    ImGui::ProgressBar(_serialProto.getBufStatus());

    ImGui::End();
}
