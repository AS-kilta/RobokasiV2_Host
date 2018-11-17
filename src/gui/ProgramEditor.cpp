#include "gui/ProgramEditor.hpp"
#include "gui/VisualizerConfig.hpp"
#include "gui/LinearDrive.hpp"

#include <imgui.h>

using namespace gui;

ProgramEditor::ProgramEditor(kin::Program& program,
                             gui::VisualizerConfig& visualizerConfig) :
    _program(program),
    _visualizerConfig(visualizerConfig)
{
    snprintf(_nextPoseName, sizeof(_nextPoseName), "Pose %zu", _program.poses.size());
    _visualizerSourceId =
            visualizerConfig.registerSource("Pose Editor",
                                            std::bind(&ProgramEditor::_poseVisualizer,
                                                      this));
}

std::array<float, 6> ProgramEditor::_poseVisualizer(void)
{
    return _angles;
}

void ProgramEditor::render()
{
    ImGui::SetNextWindowPos(ImVec2(800, 10), ImGuiSetCond_Once);
    ImGui::SetNextWindowSize(ImVec2(470, 400), ImGuiSetCond_Once);

    ImGui::Begin("Program Editor");

    ImGui::Columns(2);

    /* Pose editor */

    if (ImGui::Button("New")) {
        kin::Puma560 pose;
        for (size_t i = 0; i < 6; ++i)
            pose.setJointAngle(i, _angles[i]);

        _program.poses.push_back((kin::ProgramPose){pose, _nextPoseName});

        snprintf(_curPoseName, sizeof(_curPoseName), "%s", _program.poses[0].name.c_str());

        snprintf(_nextPoseName, sizeof(_nextPoseName), "Pose %zu", _program.poses.size());
    }
    ImGui::SameLine();
    ImGui::InputText("Name##POSE", _nextPoseName, sizeof(_nextPoseName));

    ImVec2 winSz = ImGui::GetWindowSize();

    ImGui::BeginChild("Poses", ImVec2(0, winSz.y - 266), true);

    for (size_t i = 0; i < _program.poses.size(); ++i) {
        bool isSelected = i == _selectedPoseIdx;
        auto& pose = _program.poses[i];

        if (ImGui::Selectable(pose.name.c_str(), isSelected)) {
            _selectedPoseIdx = i;
            for (size_t i = 0; i < 6; ++i)
                _angles[i] = pose.pose.getJointAngle(i);
            snprintf(_curPoseName, sizeof(_curPoseName), "%s", pose.name.c_str());
        }

        if (isSelected)
            ImGui::SetItemDefaultFocus();
    }

    ImGui::EndChild();

    if (!_program.poses.empty()) {
        auto& pose = _program.poses[_selectedPoseIdx];
        float degAngles[6];
        bool edited = 0;

        edited |= ImGui::InputText("Pose Name", _curPoseName, sizeof(_curPoseName));

        for (size_t i = 0; i < 6; ++i)
            degAngles[i] = _angles[i] / PI * 180;

        edited |= ImGui::DragFloat("j1", &degAngles[0], 0.5f, -180.0f, 180.0f);
        edited |= ImGui::DragFloat("j2", &degAngles[1], 0.5f, -180.0f, 180.0f);
        edited |= ImGui::DragFloat("j3", &degAngles[2], 0.5f, -180.0f, 180.0f);
        edited |= ImGui::DragFloat("j4", &degAngles[3], 0.5f, -180.0f, 180.0f);
        edited |= ImGui::DragFloat("j5", &degAngles[4], 0.5f, -180.0f, 180.0f);
        edited |= ImGui::DragFloat("j6", &degAngles[5], 0.5f, -180.0f, 180.0f);

        for (size_t i = 0; i < 6; ++i)
            _angles[i] = degAngles[i] / 180 * PI;

        if (edited) {
            for (size_t i = 0; i < 6; ++i)
                _program.poses[_selectedPoseIdx].pose.setJointAngle(i, _angles[i]);
            _program.poses[_selectedPoseIdx].name = _curPoseName;
            _visualizerConfig.setSource(_visualizerSourceId);
        }
    }

    ImGui::NextColumn();

    /* Step editor */

    if (_program.poses.empty()) {
        ImGui::End();
        return;
    }

    ImGui::BeginChild("New Props", ImVec2(0, 42));

    ImGui::Columns(2);

    if (ImGui::Button("New")) {
        char name[80];
        snprintf(name, sizeof(name), "Step %lu", _newStepID++);
        if (!_program.steps.empty())
            ++_selectedStepIdx;
        switch (_selectedStepType) {
        case StepTypes::LinearDriveStep:
            _program.addStep<LinearDrive>(_selectedStepIdx, name);
            break;
        default:
            break;
        }
        snprintf(_curStepName, sizeof(_curStepName), "%s", name);
    }
    ImGui::SetColumnWidth(-1, 42.0f);

    ImGui::NextColumn();

    static const char* stepTypeNames[] = {
        [StepTypes::LinearDriveStep] = "Linear drive",
    };

    if (ImGui::BeginCombo("Type", stepTypeNames[_selectedStepType])) {
        constexpr size_t n = sizeof(stepTypeNames) / sizeof(stepTypeNames[0]);
        for (size_t i = 0; i < n; ++i) {
            if (ImGui::Selectable(stepTypeNames[i], i == _selectedStepType))
                _selectedStepType = i;
            if (i == _selectedStepType)
                ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }

    ImGui::EndChild();

    winSz = ImGui::GetWindowSize();

    ImGui::BeginChild("Steps", ImVec2(0, winSz.y - 220), true);
    for (size_t i = 0; i < _program.steps.size(); ++i) {
        bool isSelected = i == _selectedStepIdx;
        auto& step = _program.steps[i];

        if (ImGui::Selectable(step->name.c_str(), isSelected)) {
            _selectedStepIdx = i;
            snprintf(_curStepName, sizeof(_curStepName), "%s", step->name.c_str());
        }

        if (isSelected)
            ImGui::SetItemDefaultFocus();
    }
    ImGui::EndChild();

    if (!_program.steps.empty()) {
        auto& step = _program.steps[_selectedStepIdx];
        if (ImGui::InputText("Name##STEP", _curStepName, sizeof(_curStepName)))
            step->name = _curStepName;
        step->edit();
        if (ImGui::BeginCombo("End pose", _program.poses[step->endPoseIdx].name.c_str())) {
            for (size_t i = 0; i < _program.poses.size(); ++i) {
                if (ImGui::Selectable(_program.poses[i].name.c_str(), i == step->endPoseIdx))
                    step->endPoseIdx = i;
                if (i == step->endPoseIdx)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
        if (ImGui::Button("Delete")) {
            _program.steps.erase(_program.steps.begin() + _selectedStepIdx);
            if (!_program.steps.empty()) {
                if (_selectedStepIdx >= _program.steps.size())
                    _selectedStepIdx =_program.steps.size() - 1;
                snprintf(_curStepName, sizeof(_curStepName), "%s",
                            _program.steps[_selectedStepIdx]->name.c_str());
            }
        }
    }

    ImGui::End();
}
