#ifndef ROBOKASIV2_HOST_HWIO_HPP
#define ROBOKASIV2_HOST_HWIO_HPP

#include <string>

#include <libserialport.h>


namespace hwio {

    class Hwio {
    public:
        Hwio(std::string portName, int baudrate);
        ~Hwio();
    private:
        struct sp_port *_port;
    };

}

#endif
