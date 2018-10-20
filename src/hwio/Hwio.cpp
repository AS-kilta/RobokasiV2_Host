#include "Hwio.hpp"


using namespace hwio;


Hwio::Hwio() :
    _port(nullptr)
{
}

int Hwio::connect(std::string portName, int baudrate)
{
    sp_return err;

    err = sp_get_port_by_name(portName.c_str(), &_port);
    if (err != SP_OK) {
        printf("Error: Could not find serial port: %s\n", portName.c_str());
        return -1;
    }
    
    err = sp_open(_port, SP_MODE_READ_WRITE);
    if (err != SP_OK) {
        printf("Error: Could not open serial port: %s\n", portName.c_str());
        return -1;
    }

    err = sp_set_baudrate(_port, baudrate);
    if (err != SP_OK) {
        printf("Error: Could not set serial port baud rate to %d\n", baudrate);
        return -1;
    }
}

Hwio::~Hwio()
{
    sp_close(_port);
}

void Hwio::_readMsg()
{
    char readBuf[512];

    do {
        /* non-blocking read up to sizeof(readBuf) 
         * copy over to _msgBuf until a newline is found,*/
        
    } while (0);
}
