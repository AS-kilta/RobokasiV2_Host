#ifndef ROBOKASIV2_HOST_SERIALCONFIG_HPP
#define ROBOKASIV2_HOST_SERIALCONFIG_HPP

#ifndef WITHOUT_LIBSERIALPORT
#include <libserialport.h>
#endif

#include <functional>


namespace gui {

    class SerialConfig {
    public:
        SerialConfig(std::string name,
                     std::function<int(std::string port, int baud)> connectCb);
        ~SerialConfig();
        void render();
    private:
#ifndef WITHOUT_LIBSERIALPORT
        struct sp_port**    _ports;
#endif
        int                 _port_idx;
        int                 _port_baud;
        std::string         _name;
        std::function<int(std::string port, int baud)> _connectCb;
    };

}

#endif
