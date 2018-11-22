#ifndef ROBOKASIV2_HOST_DRIVECONTROL_HPP
#define ROBOKASIV2_HOST_DRIVECONTROL_HPP

#include "gui/VisualizerConfig.hpp"
#include "hwio/SerialProto.hpp"
#include "hwio/CommandQueue.hpp"


namespace gui {

    class DriveControl {
    public:
        DriveControl(hwio::SerialProto& serialProto, hwio::CommandQueue& cq,
                     VisualizerConfig& visualizerConfig);
        ~DriveControl() = default;
        void render();
        kin::Puma560 getSetpoint();
    private:
        hwio::SerialProto&      _serialProto;
        hwio::CommandQueue&     _commandQueue;
        hwio::Command           _command;
        std::array<float, 6>    _ikSetpoint;
        kin::Puma560            _puma;

        VisualizerConfig& _visualizerConfig;
        size_t _sensorVisualizerId;
        size_t _setpointVisualizerId;
        std::array<float, 6> _setpointVisualizer(void);
        std::array<float, 6> _sensorVisualizer(void);

        bool _initialControlsSet;
        bool _contiguousMode;
        bool _useInverseKinematics;
    };

}

#endif
