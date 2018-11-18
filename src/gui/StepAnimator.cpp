#include "gui/StepAnimator.hpp"

#include "kinematics/Puma560.hpp"

using namespace gui;

StepAnimator::StepAnimator() :
    _frames(),
    _frameIdx(0),
    _frameDtAccum(0),
    _running(false)
{
}

void StepAnimator::prime(std::vector<kin::Puma560StepFrame>&& frames,
                        kin::Puma560 startPose)
{
    if (frames.empty())
        return;
    _frames = std::move(frames);
    _frameIdx = 0;
    _running = true;
    _prevFrame = kin::Puma560StepFrame(startPose, 0);
    _nextFrame = _frames.front();
}

void StepAnimator::feed(std::vector<kin::Puma560StepFrame>&& frames)
{
    if (!_frames.empty())
        _frameDtAccum -= _frames[_frameIdx - 1].dt;
    /*
     * Use _nextFrame from the previous step as a starting point for the new
     * step that is being fed in.
     */
    prime(std::move(frames), _nextFrame);
}

void StepAnimator::stop()
{
    _frames.clear();
    _frameIdx = 0;
    _frameDtAccum = 0;
}

bool StepAnimator::needFeed()
{
    return _frameIdx >= _frames.size() && !_running;
}

kin::Puma560 StepAnimator::_interpolate(const kin::Puma560& prevFrame,
                                        const kin::Puma560& curFrame,
                                        float r)
{
    kin::Puma560 puma;

    for (size_t i = 0; i < 6; ++i)
        puma.setJointAngle(i, prevFrame.getJointAngle(i) * (1.0f - r) +
                              curFrame.getJointAngle(i) * r);

    return puma;
}

kin::Puma560 StepAnimator::tick(uint32_t dt)
{
    kin::Puma560 puma;
    float r;

    if (needFeed()) { /* This branch should not be taken in normal operation */
        fprintf(stderr, "StepAnimator starved!\n");
        return _nextFrame;
    }

    _frameDtAccum += dt;
    
    while (_frameIdx < _frames.size() && _frameDtAccum >= (int)_nextFrame.dt) {
        if (_frameIdx != 0) {
            /*
             * The first frame (_frameIdx == 0) is a bit special, which is why
             * the state setup for it is done in feed() or prime().
             */
            _prevFrame = _nextFrame;
            _nextFrame = _frames[_frameIdx];
            _frameDtAccum -= _frames[_frameIdx].dt; /* This should be here */
        }
        ++_frameIdx;
    }

    r = float(_frameDtAccum) / _nextFrame.dt;
    if (r >= 1.0f)
        _running = false;

    return _interpolate(_prevFrame, _nextFrame, r);
}
