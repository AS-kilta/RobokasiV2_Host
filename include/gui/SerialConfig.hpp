#ifndef ROBOKASIV2_HOST_SERIALCONFIG_HPP
#define ROBOKASIV2_HOST_SERIALCONFIG_HPP

#include "hwio/SerialProto.hpp"

#ifndef WITHOUT_LIBSERIALPORT
#include <libserialport.h>
#endif


namespace gui {

    class SerialConfig {
    public:
        SerialConfig(hwio::SerialProto& serialProto);
        ~SerialConfig();
        void render();
    private:
#ifndef WITHOUT_LIBSERIALPORT
        struct sp_port**    _ports;
#endif
        int                 _port_idx;
        int                 _port_baud;
        hwio::SerialProto&  _serialProto;
    };

}

#endif
