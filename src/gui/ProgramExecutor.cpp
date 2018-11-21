#include "gui/ProgramExecutor.hpp"

#include <imgui.h>

using namespace gui;

ProgramExecutor::ProgramExecutor(const kin::Program& prog,
                                 hwio::SerialProto& proto,
                                 hwio::CommandQueue& cq) :
    _program(prog),
    _serialProto(proto),
    _commandQueue(cq)
{
}

void ProgramExecutor::render()
{
    if (!_serialProto.isConnected())
        return;


    ImGui::Begin("Program executor");

    kin::ProgramStep* curStep;

    bool bufferEmpty = _serialProto.getBufStatus() == 0 &&
                       _commandQueue.size() == 0;

    if (_state == ExecutionState::Run && bufferEmpty)
        _state = ExecutionState::Stop;

    if (_state == ExecutionState::Drain && bufferEmpty) {
        _state = ExecutionState::Pause;
        ImGui::OpenPopup("Pause dialog");
    }

    if (ImGui::BeginPopupModal("Pause dialog")) {
        bool enter = ImGui::IsKeyDown(ImGui::GetKeyIndex(ImGuiKey_Enter));
        if (ImGui::Button("Continue") || enter) {
            _state = ExecutionState::Run;
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Stop")) {
            _state = ExecutionState::Stop;
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    switch (_state) {
    case ExecutionState::Stop:
        ImGui::Text("Stopped");
        if (ImGui::Button("Run")) {
            _curStepIdx = 0;
            _state = ExecutionState::Run;
        }
        break;
    case ExecutionState::Pause:
        ImGui::Text("Paused");
        if (ImGui::Button("Run"))
            _state = ExecutionState::Run;
        break;
    case ExecutionState::Drain:
    case ExecutionState::Run:
        ImGui::Text("Running");
        break;
    default:
        break;
    }

    while (_state == ExecutionState::Run && _curStepIdx < _program.steps.size()) {
        _prevPose = _nextPose;
        curStep = _program.steps[_curStepIdx].get();
        _nextPose = _program.poses[curStep->endPoseIdx].pose;

        auto frames = curStep->generate(_prevPose, _nextPose);
        std::vector<hwio::Command> cmds;
        for (const auto frame : frames) {
            cmds.emplace_back(frame, frame.dt);
            cmds.back().safemode = false;
        }

        if (curStep->shouldPause)
            _state = ExecutionState::Drain;

        _commandQueue.addCommands(cmds);

        ++_curStepIdx;
    }

    ImGui::End();
}

void ProgramExecutor::drain()
{
    _state = ExecutionState::Drain;
}
