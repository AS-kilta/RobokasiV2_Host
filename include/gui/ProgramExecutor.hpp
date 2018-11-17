#ifndef ROBOKASIV2_GUI_PROGRAMEXECUTOR_HPP
#define ROBOKASIV2_GUI_PROGRAMEXECUTOR_HPP

#include "kinematics/Program.hpp"
#include "hwio/CommandQueue.hpp"
#include "hwio/SerialProto.hpp"

namespace gui {
    class ProgramExecutor {
    public:
        ProgramExecutor(const kin::Program& prog,
                        hwio::SerialProto& proto,
                        hwio::CommandQueue& cq);
        void render();
        void drain();

    private:
        enum ExecutionState {
            Stop,
            Run,
            Drain,
            Pause,
        };
        int _state = ExecutionState::Stop;
        const kin::Program& _program;
        hwio::SerialProto&  _serialProto;
        hwio::CommandQueue& _commandQueue;
        size_t _curStepIdx;
        kin::Puma560 _startPose;
        kin::Puma560 _prevPose;
        kin::Puma560 _nextPose;
    };
}

#endif
