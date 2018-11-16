#include "gui/ProgramEditor.hpp"
#include "gui/VisualizerConfig.hpp"
#include "gui/LinearDrive.hpp"

#include <imgui.h>

using namespace gui;

static void copyStringToCharVec(std::vector<char>& vec, std::string str)
{
    vec.clear();
    std::copy(begin(str), end(str), back_inserter(vec));
    vec.push_back('\0');
}

ProgramEditor::ProgramEditor(kin::Program& program,
                             gui::VisualizerConfig& visualizerConfig) :
    _program(program),
    _nextPoseName(80),
    _curPoseName(80),
    _newStepName(80)
{
    snprintf(_nextPoseName.data(), _nextPoseName.capacity(), "Pose %zu",
             _program.poses.size());
    snprintf(_newStepName.data(), _newStepName.capacity(), "Step %zu",
             _program.steps.size());
    visualizerConfig.registerSource("Pose Editor",
                                    std::bind(&ProgramEditor::_poseVisualizer, this));
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

        _program.poses.push_back((kin::ProgramPose){pose, _nextPoseName.data()});

        if (_program.poses.size() == 1)
            copyStringToCharVec(_curPoseName, _program.poses[0].name);

        snprintf(_nextPoseName.data(), _nextPoseName.capacity(), "Pose %zu",
                 _program.poses.size());
    }
    ImGui::SameLine();
    ImGui::InputText("Name", _nextPoseName.data(), _nextPoseName.capacity());

    ImVec2 winSz = ImGui::GetWindowSize();

    ImGui::BeginChild("Poses", ImVec2(0, winSz.y - 266), true);

    for (size_t i = 0; i < _program.poses.size(); ++i) {
        bool isSelected = i == _selectedPoseIdx;
        auto& pose = _program.poses[i];

        if (ImGui::Selectable(pose.name.c_str(), isSelected)) {
            _selectedPoseIdx = i;
            for (size_t i = 0; i < 6; ++i)
                _angles[i] = pose.pose.getJointAngle(i);
            copyStringToCharVec(_curPoseName, pose.name);
        }

        if (isSelected)
            ImGui::SetItemDefaultFocus();
    }

    ImGui::EndChild();

    if (!_program.poses.empty()) {
        auto& pose = _program.poses[_selectedPoseIdx];
        float degAngles[6];
        bool edited = 0;

        edited |= ImGui::InputText("Pose Name", _curPoseName.data(), _curPoseName.capacity());

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
            _program.poses[_selectedPoseIdx].name = _curPoseName.data();
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
        switch (_selectedStepType) {
        case StepTypes::LinearDriveStep:
            _program.addStep<LinearDrive>(std::string(_newStepName.data()));
            break;
        default:
            break;
        }
        snprintf(_newStepName.data(), _newStepName.capacity(), "Step %zu",
                 _program.steps.size());
    }
    ImGui::SetColumnWidth(-1, 42.0f);

    ImGui::NextColumn();
    ImGui::InputText("Name", _newStepName.data(), _newStepName.capacity());

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
            copyStringToCharVec(_curStepName, step->name);
        }

        if (isSelected)
            ImGui::SetItemDefaultFocus();
    }
    ImGui::EndChild();

    if (!_program.steps.empty()) {
        auto& step = _program.steps[_selectedStepIdx];
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
    }

    ImGui::End();
}
