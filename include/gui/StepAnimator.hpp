#ifndef ROBOKASIV2_GUI_STEPANIMATOR_HPP
#define ROBOKASIV2_GUI_STEPANIMATOR_HPP

#include "kinematics/Program.hpp"

namespace gui {
    class StepAnimator {
    public:
        StepAnimator();
        /* 
         * When starting a new program sequence the animator should be
         * initialized by feeding in the first step with prime(). The subsequent
         * steps of the animation should be fed in with feed(). Once the end of
         * the program is reached the animation should be stopped with stop().
         *
         * Starvation is a condition which is reached if the animator runs out
         * of frames unexpectedly. The risk of starvation is signaled by
         * needFeed() returning true and starvation should be avoided by either
         * stopping the animator once the end of an animation sequence has been
         * reached or by feeding the animator before the next animator tick().
         *
         * The state transitions marked "-" are either undefined or unreachable
         * and can result in weird behaviour.
         *
         * state / event | prime() | feed()  | stop()  | needFeed() == 1 |
         * --------------+---------+---------+---------+-----------------+
         *  stopped      | running | -       | stopped | -               |
         *  running      | -       | running | stopped | starved         |
         *  starved      | running | running | stopped | starved         |
         */
        void prime(std::vector<kin::Puma560StepFrame>&& frames,
                  kin::Puma560 startPose);
        void feed(std::vector<kin::Puma560StepFrame>&& frames);
        bool needFeed();
        void stop();
        kin::Puma560 tick(uint32_t dt);
    private:
        std::vector<kin::Puma560StepFrame> _frames;
        size_t _frameIdx;
        uint32_t _frameDtAccum;
        bool _running;
        kin::Puma560StepFrame _prevFrame;
        kin::Puma560StepFrame _nextFrame;
        kin::Puma560 _interpolate(const kin::Puma560& prevFrame,
                                  const kin::Puma560& curFrame,
                                  float r);
    };
};

#endif
