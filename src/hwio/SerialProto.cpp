#include "SerialProto.hpp"

#include <chrono>

#ifndef WITHOUT_LIBSERIALPORT
#include <libserialport.h>
#endif


using namespace hwio;


Command::Command(const State& state, int dt) :
    angles(state.angles),
    gripper(state.gripper),
    brake(state.brake),
    safemode(state.safemode),
    dt(dt)
{
}

Command::Command(const State& state) :
    Command::Command(state, 16)
{
}

Command::Command(const kin::Puma560& puma, int dt) :
    gripper(puma.gripper),
    brake(false),
    safemode(false),
    dt(dt)
{
    int i;
    for (i = 0; i < 6; ++i)
        angles[i] = puma.getJointAngle(i);

    if (angles[2] > PI/2) angles[2] -= 2.0f*PI;
}

Command::Command(const kin::Puma560& puma, const Command& c) :
    gripper(c.gripper),
    brake(c.brake),
    safemode(c.safemode),
    dt(c.dt)
{
    int i;
    for (i = 0; i < 6; ++i)
        angles[i] = puma.getJointAngle(i);

    if (angles[2] > PI/2) angles[2] -= 2.0f*PI;
}

SerialProto::SerialProto() :
#ifndef WITHOUT_LIBSERIALPORT
    _port(nullptr),
#endif
    _connected(false),
    _pollThreadRunning(true),
    _pollThread(std::bind(&SerialProto::_pollThreadFunc, this)),
    _bufUsed(0),
    _bufCapa(1),
    _stateMsgFmt("angle: j1: %f, j2: %f, j3: %f, j4: %f, j5: %f, j6: %f, safemode: %i, brake: %i, gripper: %i, dt: %i\n"),
    _bufStatusRespFmt("buffer used: %u/%u\r\n")
{
}

SerialProto::~SerialProto()
{
    _pollThreadRunning = false;

    _pollThread.join();

#ifndef WITHOUT_LIBSERIALPORT
    if (_port)
        sp_close(_port);
#endif
}

int SerialProto::connect(std::string portName, int baudrate)
{
#ifndef WITHOUT_LIBSERIALPORT
    sp_return err;

    {
        std::lock_guard<std::mutex> guard(_portLock);

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
            printf("Error: Could not set serial port baud rate to %d\n",
                   baudrate);
            return -1;
        }

        _connected = true;
    }

    return _updateState();
#else
    return 0;
#endif
}

bool SerialProto::isConnected()
{
    return _connected;
}

State SerialProto::getState()
{
    std::lock_guard<std::mutex> guard(_stateLock);
    return _state;
}

float SerialProto::getBufStatus()
{
    std::lock_guard<std::mutex> guard(_bufStatusLock);
    return (float)_bufUsed / _bufCapa;
}

int SerialProto::sendCommand(const Command& cmd)
{
    std::string resp;
    char msg[512];
    constexpr int maxRetries = 3;
    int retries = maxRetries - 1;
    int ret;

    do {
        if (retries != maxRetries - 1)
            printf("Retrying...\n");

        snprintf(msg, sizeof(msg), _stateMsgFmt, cmd.angles[0], cmd.angles[1],
                 cmd.angles[2], cmd.angles[3], cmd.angles[4], cmd.angles[5],
                 cmd.safemode, cmd.brake, cmd.gripper, cmd.dt);

        if ((msg[0] != '\n') && (msg[0] = 'b')) {
            printf("%s\n", msg);
        }

        ret = _doTransaction(msg, resp);
        if (ret)
            continue;

        {
            std::lock_guard<std::mutex> guard(_bufStatusLock);
            ret = sscanf(resp.c_str(), _bufStatusRespFmt, &_bufUsed, &_bufCapa);
        }
        if (ret == 2) {
            return 0;
        } else if (!resp.compare("buffer overrun\r\n")) {
            fprintf(stderr, "Buffer overrun!\n");
            return -1;
        } else {
            fprintf(stderr, "Failed to parse buffer state response: \"%s\"\n",
                    resp.c_str());
            continue;
        }
    } while (--retries);

    return -1;
}

void SerialProto::registerBufNotifyCb(float notifyThreshold, BufNotifyCbType cb)
{
    _bufNotifyThreshold = notifyThreshold;
    _bufNotifyCb = cb;
}

int SerialProto::_doTransaction(const std::string& msg, std::string& resp)
{
#ifndef WITHOUT_LIBSERIALPORT
    std::lock_guard<std::mutex> guard(_portLock);
    enum sp_return sp_ret;
    char c;

    sp_ret = sp_flush(_port, SP_BUF_BOTH);
    if (sp_ret < 0) {
        fprintf(stderr, "Serial port flush failed\n");
        _connected = false;
        return -1;
    }

    sp_ret = sp_blocking_write(_port, msg.c_str(), msg.size(), 100);
    if (sp_ret < 0) {
        fprintf(stderr, "Serial port write failed\n");
        _connected = false;
        return -1;
    }

    resp.clear();
    do {
        sp_ret = sp_blocking_read(_port, &c, 1, 100);
        if (sp_ret <= 0) {
            fprintf(stderr, "Serial port read failed: %d\n", sp_ret);
            _connected = false;
            return -1;
        }
        resp.push_back(c);
    } while (resp.back() != '\n');
#endif

    return 0;
}

int SerialProto::_updateBufStatus()
{
    const std::string msg = "b\n";
    std::string resp;
    constexpr int maxRetries = 3;
    int retries = maxRetries - 1;
    int ret;

    do {
        if (retries != maxRetries - 1)
            printf("Retrying...\n");

        ret = _doTransaction(msg, resp);
        if (ret)
            continue;

        {
            std::lock_guard<std::mutex> guard(_bufStatusLock);
            ret = sscanf(resp.c_str(), _bufStatusRespFmt, &_bufUsed, &_bufCapa);
        }

        if (ret != 2) {
            fprintf(stderr, "Failed to parse buffer status response: \"%s\"\n",
                    resp.c_str());
            continue;
        }

        return 0;
    } while (--retries);

    _connected = false;
    return 1;
}

int SerialProto::_updateState()
{
    const std::string msg = "\n";
    std::string resp;
    constexpr int maxRetries = 3;
    int retries = maxRetries - 1;
    int ret;

    do {
        if (retries != maxRetries - 1)
            printf("Retrying...\n");

        ret = _doTransaction(msg, resp);
        if (ret)
            continue;

        {
            int dummy; /* This should be removed from the protocol itself */

            std::lock_guard<std::mutex> guard(_stateLock);
            ret = sscanf(resp.c_str(), _stateMsgFmt,
                         &_state.angles[0], &_state.angles[1], &_state.angles[2],
                         &_state.angles[3], &_state.angles[4], &_state.angles[5],
                         &_state.safemode, &_state.brake, &_state.gripper, &dummy);
        }

        if (ret != 10) {
            fprintf(stderr, "Failed to parse state response: \"%s\"\n",
                    resp.c_str());
            continue;
        }

        return 0;
    } while (--retries);

    _connected = false;
    return 1;
}

void SerialProto::_pollThreadFunc()
{
    using namespace std::chrono_literals;

    while (_pollThreadRunning) {
        /* Periodically update the state */
        if (_connected) {
            _updateState();
            _updateBufStatus();

            if (getBufStatus() <= _bufNotifyThreshold)
                if (_bufNotifyCb)
                    _bufNotifyCb(_bufUsed, _bufCapa);
        }
        std::this_thread::sleep_for(100ms);
    }
}
