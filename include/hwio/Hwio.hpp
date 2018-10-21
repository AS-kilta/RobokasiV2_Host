#ifndef ROBOKASIV2_HOST_HWIO_HPP
#define ROBOKASIV2_HOST_HWIO_HPP

#include <array>
#include <string>
#include <vector>
#include <atomic>
#include <thread>

#include <libserialport.h>


namespace hwio {

    struct State {
        std::array<float, 6> angles;
        bool gripper = 0;
        bool brake = 1;
        bool safemode = 1;
        int dt = 16;
    };

    class HwioBuffer {
    public:
        HwioBuffer();
        ~HwioBuffer();
        void threadFunc();
    private:
        static void _threadFuncWrapper(HwioBuffer* obj)
        {
            obj->threadFunc();
        };
        std::atomic<bool> _threadRunning;
        std::thread _thread;
    };

    class Hwio {
    /*
     * Serial port state
     * Protocol translation
     */
    public:
        Hwio();
        ~Hwio();
        int updateState();
        bool isConnected();
        int setState(const State& state);
        int getAngles(std::array<float, 6>& angles);
        int connect(std::string portName, int baudrate);
    private:
        HwioBuffer _hwioBuffer;
        struct sp_port* _port;
        void _readMsg();
        std::string _msgBuf;
        bool _connected;
        const char *_stateMsgFmt = "angle: j1: %f, j2: %f, j3: %f, j4: %f, j5: %f, j6: %f, safemode: %i, brake: %i, gripper: %i, dt: %i\r\n";
        State _curState;
        State _setState;
    };

    class SerialProto {
    };
}

#endif
