#ifndef ROBOKASIV2_HOST_HWIO_HPP
#define ROBOKASIV2_HOST_HWIO_HPP

#include <string>
#include <vector>

#include <libserialport.h>


namespace hwio {

    class Hwio {
    public:
        Hwio();
        ~Hwio();
        int connect(std::string portName, int baudrate);
    private:
        struct sp_port* _port;
        void _readMsg();
        std::string _msgBuf;
    };

}

#endif
