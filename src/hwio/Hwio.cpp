#include <string.h>
#include <math.h>

#include "Hwio.hpp"


using namespace hwio;

HwioBuffer::HwioBuffer() :
    _threadRunning(true),
    _thread(HwioBuffer::_threadFuncWrapper, this)
{
}

HwioBuffer::~HwioBuffer()
{
    _threadRunning = false;
    _thread.join();
}

void HwioBuffer::threadFunc()
{
    while (_threadRunning) {
        printf("moi\n");
    }
}


Hwio::Hwio() :
    _hwioBuffer(),
    _port(nullptr),
    _connected(false)
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

    _connected = true;

    return 0;
}

int Hwio::updateState()
{
    char readBuf[512] = { '\0' };
    char msg[] = "\r\n";
    size_t i = 0;
    enum sp_return sp_ret;
    int retries = 3;
    int ret;

    for (int retries = 3; retries > 0; --retries) {
        sp_flush(_port,	SP_BUF_BOTH);
        sp_blocking_write(_port, msg, sizeof(msg) - 1, 1000);
        i = 0;
        do {
            sp_ret = sp_blocking_read(_port, readBuf + i, 1, 1000);
            if (sp_ret < 0)
                return sp_ret;
            i += sp_ret;
        } while (readBuf[i - 1] != '\n' && i < sizeof(readBuf) - 1);

        readBuf[i] = '\0';

        if (!strcmp("invalid command\r\n", readBuf)) {
            printf("invalid\n");
            continue;
        }

        ret = sscanf(readBuf, _stateMsgFmt, &_curState.angles[0],
                     &_curState.angles[1], &_curState.angles[2],
                     &_curState.angles[3], &_curState.angles[4],
                     &_curState.angles[5], &_curState.safemode,
                     &_curState.brake, &_curState.gripper, &_curState.dt);
        if (ret == 10)
            return 0;
    };

    return -1;
}

int Hwio::getAngles(std::array<float, 6> &angles)
{
    /* TODO test if cur state too old */

    for (size_t i = 0; i < 6; ++i)
        angles[i] = _curState.angles[i];

    return 0;
}

int Hwio::setState(const State& state)
{
    enum sp_return sp_ret;
    size_t i = 0;
    char writeBuf[512];
    char readBuf[512];

    /* XXX Is _setState needed at all? */
    //_setState = state;

    snprintf(writeBuf, sizeof(writeBuf), _stateMsgFmt, state.angles[0],
             state.angles[1], state.angles[2], state.angles[3],
             state.angles[4], state.angles[5], state.safemode, state.brake,
             state.gripper, 16);
    sp_blocking_write(_port, writeBuf, strlen(writeBuf), 1000);

    do {
        sp_ret = sp_blocking_read(_port, readBuf + i, 1, 1000);
        if (sp_ret < 0)
            return sp_ret;
        i += sp_ret;
    } while (readBuf[i - 1] != '\n' && i < sizeof(readBuf) - 1);

    readBuf[i] = '\0';
}

Hwio::~Hwio()
{
    sp_close(_port);
}

bool Hwio::isConnected()
{
    return _connected;
}

void Hwio::_readMsg()
{
    char readBuf[512];

    do {
        /* non-blocking read up to sizeof(readBuf) 
         * copy over to _msgBuf until a newline is found,*/
        
    } while (0);
}
