#include "gui/VisualizerConfig.hpp"
#include "gui/Puma560Model.hpp"
#include "kinematics/MathTypes.hpp"

#include <imgui.h>

using namespace gui;

size_t VisualizerConfig::registerSource(std::string name,
                                        std::function<std::array<float, 6>(void)> fn)
{
    size_t id = _sources.size();
    _sources.emplace_back((_VisualizerSource) { name, fn });
    return id;
}

void VisualizerConfig::setSource(size_t id)
{
    if (id >= _sources.size())
        return;
    _curSourceId = id;
}

void VisualizerConfig::render(std::shared_ptr<gui::Puma560Model> model)
{
    if (_sources.empty())
        return;

    ImGui::SetNextWindowPos(ImVec2(10, 610), ImGuiSetCond_Once);
    ImGui::SetNextWindowSize(ImVec2(260, 100), ImGuiSetCond_Once);

    ImGui::Begin("Visualizer");

    ImGui::Checkbox("Mesh", &meshRenderEnable);
    ImGui::SameLine();
    ImGui::Checkbox("Frames", &frameRenderEnable);

    if (ImGui::BeginCombo("Source", _sources[_curSourceId].name.c_str())) {
        for (size_t i = 0; i < _sources.size(); ++i) {
            if (ImGui::Selectable(_sources[i].name.c_str(), i == _curSourceId))
                _curSourceId = i;
            if (i == _curSourceId)
                ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }

    std::array<float, 6> angles = _sources[_curSourceId].fn();

    for (int i = 0; i < 6; ++i)
        model->setJointAngle(i, angles[i]);

    ImGui::End();
}
