#include "Hwio.hpp"


using namespace hwio;


Hwio::Hwio(std::string portName, int baudrate) :
    _port (nullptr)
{
    sp_return err;

    err = sp_get_port_by_name(portName.c_str(), &_port);
    if (err != SP_OK) {
        printf("Error: Colud not find serial port: %s\n", portName.c_str());
        return;
    }
    
    err = sp_open(_port, SP_MODE_READ_WRITE);
    if (err != SP_OK) {
        printf("Error: Could not open serial port: %s\n", portName.c_str());
        return;
    }

    err = sp_set_baudrate(_port, baudrate);
    if (err != SP_OK) {
        printf("Error: Could not set serial port baud rate to %d\n", baudrate);
        return;
    }
}

Hwio::~Hwio()
{
    sp_close(_port);
}
