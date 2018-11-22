#include "gui/ProgramEditor.hpp"
#include "gui/VisualizerConfig.hpp"
#include "gui/LinearDrive.hpp"
#include "gui/StepTypes.hpp"
#include "hwio/SerialProto.hpp"

#include <imgui.h>

#include <json.hpp>

#include <fstream>
#include <iomanip>

using json = nlohmann::json;
using namespace gui;

ProgramEditor::ProgramEditor(kin::Program& program,
                             hwio::SerialProto& serialProto,
                             gui::VisualizerConfig& visualizerConfig) :
    _program(program),
    _serialProto(serialProto),
    _visualizerConfig(visualizerConfig)
{
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

    ImGui::Begin("Program Editor", NULL, ImGuiWindowFlags_MenuBar);

    if (ImGui::BeginMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            /* TODO Add a file browser dialog */
            if (ImGui::MenuItem("Save"))
                _save("test.json");
            if (ImGui::MenuItem("Load"))
                _load("test.json");
            ImGui::EndMenu();
        }
        ImGui::EndMenuBar();
    }

    ImGui::Columns(2);

    /* Pose editor */

    int insertedPose = -1;
    int removedPose = -1;

    if (ImGui::Button("New##POSE")) {
        // Insert new pose after currently selected one
        kin::Puma560 pose;
        for (size_t i = 0; i < 6; ++i)
            pose.setJointAngle(i, _angles[i]);

        char name[80];
        snprintf(name, sizeof(name), "Pose %lu", _newPoseID++);

        if (!_program.poses.empty())
            ++_selectedPoseIdx;
        if (!_program.poses.empty() && _selectedPoseIdx < _program.poses.size())
            _program.poses.insert(_program.poses.begin() + _selectedPoseIdx,
                                  {pose, name});
        else
            _program.poses.push_back({pose, name});

        snprintf(_curPoseName, sizeof(_curPoseName), "%s", name);
        insertedPose = _selectedPoseIdx;
    }

    ImVec2 winSz = ImGui::GetWindowSize();

    ImGui::BeginChild("Poses", ImVec2(0, winSz.y - 300), true);

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
        auto& currentPose = _program.poses[_selectedPoseIdx];
        float degAngles[6];
        bool edited = 0;

        edited |= ImGui::InputText("Name##POSE", _curPoseName, sizeof(_curPoseName));

        for (size_t i = 0; i < 6; ++i)
            degAngles[i] = _angles[i] / PI * 180;

        edited |= ImGui::DragFloat("j1", &degAngles[0], 0.5f, -180.0f, 180.0f);
        edited |= ImGui::DragFloat("j2", &degAngles[1], 0.5f, -180.0f, 180.0f);
        edited |= ImGui::DragFloat("j3", &degAngles[2], 0.5f, -180.0f, 180.0f);
        edited |= ImGui::DragFloat("j4", &degAngles[3], 0.5f, -180.0f, 180.0f);
        edited |= ImGui::DragFloat("j5", &degAngles[4], 0.5f, -180.0f, 180.0f);
        edited |= ImGui::DragFloat("j6", &degAngles[5], 0.5f, -180.0f, 180.0f);
        edited |= ImGui::Checkbox("gripper", &currentPose.pose.gripper);

        for (size_t i = 0; i < 6; ++i)
            _angles[i] = degAngles[i] / 180 * PI;

        if (ImGui::Button("Delete##POSE")) {
            removedPose = _selectedPoseIdx;
            _program.poses.erase(_program.poses.begin() + _selectedPoseIdx);
            if (!_program.poses.empty()) {
                if (_selectedPoseIdx >= _program.poses.size())
                    _selectedPoseIdx =_program.poses.size() - 1;
                snprintf(_curPoseName, sizeof(_curPoseName), "%s",
                         _program.poses[_selectedPoseIdx].name.c_str());
            }
        }
        ImGui::SameLine();
        if (_serialProto.isConnected() && ImGui::Button("HW Sensors##POSE")) {
            hwio::State state = _serialProto.getState();
            _angles = state.angles;
            edited = true;
        }

        if (edited) {
            for (size_t i = 0; i < 6; ++i)
                currentPose.pose.setJointAngle(i, _angles[i]);
            currentPose.name = _curPoseName;
            _visualizerConfig.setSource(_visualizerSourceId);
        }

    }

    ImGui::NextColumn();

    // Update steps if a pose was added or removed
    if (!_program.steps.empty()) {
        if (insertedPose != -1) {
            // Point steps to correct poses
            for (auto& step : _program.steps) {
                if (step->endPoseIdx >= (size_t)insertedPose)
                    ++step->endPoseIdx;
            }
        }
        if (removedPose != -1) {
            for (size_t i = 0; i < _program.steps.size();) {
                if (_program.steps[i]->endPoseIdx == (size_t)removedPose) {
                    // Remove step pointing to removed pose and update selection
                    removeStep(i);
                    if (_selectedStepIdx > i)
                        --_selectedStepIdx;
                    continue;
                } else if (_program.steps[i]->endPoseIdx > (size_t)removedPose) {
                    // Point step to correct pose
                    --_program.steps[i]->endPoseIdx;
                }
                // Increment here so that removal doesn't skip over a step
                ++i;
            }
        }
    }

    /* Step editor */

    if (_program.poses.empty()) {
        ImGui::End();
        return;
    }

    ImGui::BeginGroup();

    if (ImGui::Button("New##STEP")) {
        char name[80];
        snprintf(name, sizeof(name), "Step %lu", _newStepID++);
        if (!_program.steps.empty())
            ++_selectedStepIdx;
        switch (_selectedStepType) {
        case StepTypes::LinearDriveStep:
            _program.addStep<LinearDrive>(_selectedStepIdx, name, _selectedPoseIdx);
            break;
        case StepTypes::PauseProgramStep:
            _program.addStep<PauseProgram>(name, _selectedPoseIdx);
            break;
        default:
            break;
        }
        snprintf(_curStepName, sizeof(_curStepName), "%s", name);
    }

    ImGui::SameLine();
    if (ImGui::BeginCombo("Type", stepTypeNames[_selectedStepType])) {
        for (size_t i = 0; i < numStepTypeNames; ++i) {
            if (ImGui::Selectable(stepTypeNames[i], i == (size_t)_selectedStepType))
                _selectedStepType = i;
            if (i == (size_t)_selectedStepType)
                ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }

    ImGui::EndGroup();

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
        if (ImGui::Button("Delete##STEP"))
            removeStep(_selectedStepIdx);
        ImGui::SameLine();
        if (ImGui::ArrowButton("##SwapStepUp", ImGuiDir_Up)) {
            if (_selectedStepIdx > 0) {
                auto stepIter = _program.steps.begin();
                std::iter_swap(stepIter + _selectedStepIdx - 1, stepIter + _selectedStepIdx);
                --_selectedStepIdx;
            }
        }
        ImGui::SameLine();
        if (ImGui::ArrowButton("##SwapStepDown", ImGuiDir_Down)) {
            if (_selectedStepIdx < _program.steps.size() - 1) {
                auto stepIter = _program.steps.begin();
                std::iter_swap(stepIter + _selectedStepIdx, stepIter + _selectedStepIdx + 1);
                ++_selectedStepIdx;
            }
        }
    }

    ImGui::End();
}

void ProgramEditor::removeStep(size_t i)
{
    _program.steps.erase(_program.steps.begin() + i);
    if (i == _selectedStepIdx) {
        if (!_program.steps.empty()) {
            if (_selectedStepIdx >= _program.steps.size())
                _selectedStepIdx =_program.steps.size() - 1;
            snprintf(_curStepName, sizeof(_curStepName), "%s",
                        _program.steps[_selectedStepIdx]->name.c_str());
        }
    }
}

void ProgramEditor::_save(const char *path)
{
    json j{{"poses", _program.poses}, {"steps", _program.steps}};
    std::ofstream file(path);

    file << std::setw(4) << j;
}

void ProgramEditor::_load(const char *path)
{
    std::ifstream file(path);
    json j = json::parse(file);
    size_t parseID;

    _program.poses.clear();
    for (size_t i = 0; i < j["poses"].size(); ++i) {
        _program.poses.emplace_back(j["poses"][i]);

        /* Avoid name collisions on default names */
        if (sscanf(_program.poses.back().name.c_str(), "Pose %zu", &parseID) == 1)
            if (parseID >= _newPoseID)
                _newPoseID = parseID + 1;
    }

    _program.steps.clear();
    for (size_t i = 0; i < j["steps"].size(); ++i) {
        json& step = j["steps"][i];
        const std::string typeName = step["typeName"].get<std::string>();

        switch (getStepTypeNameIdx(typeName.c_str())) {
        case StepTypes::LinearDriveStep:
            _program.addStep<gui::LinearDrive>(step);
            break;
        case StepTypes::PauseProgramStep:
            _program.addStep<gui::PauseProgram>(step);
            break;
        default:
            break;
        }

        /* Avoid name collisions on default names */
        if (sscanf(_program.steps.back()->name.c_str(), "Step %zu", &parseID) == 1)
            if (parseID >= _newStepID)
                _newStepID = parseID + 1;
    }
}
