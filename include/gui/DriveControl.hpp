#ifndef ROBOKASIV2_HOST_DRIVECONTROL_HPP
#define ROBOKASIV2_HOST_DRIVECONTROL_HPP

#include "hwio/SerialProto.hpp"

#include <libserialport.h>


namespace gui {

    class DriveControl {
    public:
        DriveControl(hwio::SerialProto& serialProto);
        ~DriveControl() = default;
        void render();
    private:
        hwio::SerialProto&  _serialProto;
        hwio::Command       _command;
        bool                _initialControlsSet;
        bool                _contiguousMode;
    };

}

#endif
