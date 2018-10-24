#ifndef ROBOKASIV2_HOST_SERIALCONFIG_HPP
#define ROBOKASIV2_HOST_SERIALCONFIG_HPP

#include "hwio/SerialProto.hpp"

#include <libserialport.h>


namespace gui {

    class SerialConfig {
    public:
        SerialConfig(hwio::SerialProto& serialProto);
        ~SerialConfig();
        void render();
    private:
        struct sp_port**    _ports;
        int                 _port_idx;
        int                 _port_baud;
        hwio::SerialProto&  _serialProto;
    };

}

#endif
