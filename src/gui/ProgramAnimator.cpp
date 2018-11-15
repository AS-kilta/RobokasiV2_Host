#include "gui/ProgramAnimator.hpp"
#include "gui/StepAnimator.hpp"

#include "kinematics/Puma560.hpp"

#include <imgui.h>

using namespace gui;

ProgramAnimator::ProgramAnimator(const kin::Program& prog,
                                 VisualizerConfig& visualizerConfig) :
    _program(prog),
    _visualizerConfig(visualizerConfig),
    _curStepIdx(0),
    _stepFrameDtAccum(0),
    _stepAnimator()
{
    visualizerConfig.registerSource("Program Animator",
                                    std::bind(&ProgramAnimator::_visualizer, this));
}

void ProgramAnimator::render(uint32_t dt)
{
    if (_program.steps.empty())
        return;

    ImGui::SetNextWindowPos(ImVec2(1090, 650), ImGuiSetCond_Once);
    ImGui::SetNextWindowSize(ImVec2(180, 60), ImGuiSetCond_Once);

    ImGui::Begin("Program Animator");

    kin::ProgramStep* curStep;

    switch (_animationState) {
    case AnimationState::Stop:
        if (ImGui::Button("Run")) {
            _curStepIdx = 0;

            _prevPose = _startPose; /* TODO Add controls for this */

            curStep = _program.steps[_curStepIdx].get();
            _nextPose = _program.poses[curStep->endPoseIdx].pose;
            _stepAnimator.prime(curStep->generate(_prevPose, _nextPose), _prevPose);
            _animationState = AnimationState::Run;
        }
        break;
    case AnimationState::Run:
        if (ImGui::Button("Pause"))
            _animationState = AnimationState::Pause;
        break;
    case AnimationState::Pause:
        if (ImGui::Button("Run"))
            _animationState = AnimationState::Run;
        break;
    default:
        break;
    }

    ImGui::SameLine();

    if (ImGui::Button("Stop")) {
        _animationState = AnimationState::Stop;
        _stepAnimator.stop();
    }

    if (_animationState == AnimationState::Run) {
        if (_stepAnimator.needFeed()) {
            ++_curStepIdx;
            if (_curStepIdx >= _program.steps.size()) {
                _animationState = AnimationState::Stop;
                _stepAnimator.stop();
                ImGui::End();
                return;
            }
            curStep = _program.steps[_curStepIdx].get();
            _prevPose = _nextPose;
            _nextPose = _program.poses[curStep->endPoseIdx].pose;
            _stepAnimator.feed(curStep->generate(_prevPose, _nextPose));
        }

        auto frame = _stepAnimator.tick(dt);
        for (size_t i = 0; i < 6; ++i)
            _angles[i] = frame.getJointAngle(i);
    }

    ImGui::End();
}

std::array<float, 6> ProgramAnimator::_visualizer()
{
    return _angles;
}
