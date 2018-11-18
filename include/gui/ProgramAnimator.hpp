#ifndef ROBOKASIV2_GUI_PROGRAMANIMATOR_HPP
#define ROBOKASIV2_GUI_PROGRAMANIMATOR_HPP

#include "kinematics/Program.hpp"
#include "gui/VisualizerConfig.hpp"
#include "gui/StepAnimator.hpp"

namespace gui {
    class ProgramAnimator {
    public:
        ProgramAnimator(const kin::Program& prog,
                        VisualizerConfig& visualizerConfig);
        void render(uint32_t dt);
    private:
        const kin::Program& _program;
        VisualizerConfig& _visualizerConfig;
        size_t _visualizerSourceId;
        enum AnimationState {
            Stop,
            Run,
            Pause,
        };
        int _animationState = AnimationState::Stop;
        size_t _curStepIdx;
        kin::Puma560 _startPose;
        kin::Puma560 _prevPose;
        kin::Puma560 _nextPose;
        std::array<float, 6> _angles = {0};
        std::array<float, 6> _visualizer();
        StepAnimator _stepAnimator;
    };
}

#endif
