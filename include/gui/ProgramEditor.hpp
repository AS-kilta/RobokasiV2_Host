#ifndef ROBOKASIV2_GUI_PROGRAMEDITOR_HPP
#define ROBOKASIV2_GUI_PROGRAMEDITOR_HPP

#include "kinematics/Program.hpp"
#include "gui/VisualizerConfig.hpp"

#include <imgui.h>

namespace gui {

    class ProgramEditor {
    public:
        ProgramEditor(kin::Program& program,
                      gui::VisualizerConfig& visualizerConfig);
        void render();
    private:
        kin::Program& _program;

        std::array<float, 6> _angles = {0.0f};
        std::array<float, 6> _poseVisualizer(void);

        gui::VisualizerConfig&  _visualizerConfig;
        size_t                  _visualizerSourceId;

        /* Pose editor */
        size_t _selectedPoseIdx = 0;

        /* ImGui doesn't deal with std::string */
        std::vector<char> _nextPoseName;
        std::vector<char> _curPoseName;

        /* Sequence editor */
        enum StepTypes {
            LinearDriveStep,
        };
        int _selectedStepType = StepTypes::LinearDriveStep;

        size_t _selectedStepIdx = 0;

        std::vector<char> _newStepName;
        std::vector<char> _curStepName;
    };

};

#endif
