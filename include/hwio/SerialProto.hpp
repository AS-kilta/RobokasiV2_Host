#ifndef ROBOKASIV2_HOST_SERIALPROTO_HPP
#define ROBOKASIV2_HOST_SERIALPROTO_HPP

#include "kinematics/Puma560.hpp"

#include <array>
#include <string>
#include <atomic>
#include <thread>
#include <functional>
#include <mutex>

#ifndef WITHOUT_LIBSERIALPORT
#include <libserialport.h>
#endif


namespace hwio {

    struct State {
        std::array<float, 6> angles;
        std::array<float, 6> current;
        /* scanf doesn't know about bool */
        int gripper;
        int brake;
        int safemode;
    };

    struct Command {
        Command() = default;
        Command(const State& state, int dt);
        Command(const State& state);
        Command(const kin::Puma560& puma, int dt);
        std::array<float, 6> angles = { 0.5f, 0.5f,  0.5f, 0.5f,  0.5f, 0.5f };
        bool gripper = 0;
        bool brake = 1;
        bool safemode = 1;
        int dt = 16;
    };

    class SerialProto {
    public:
        using BufNotifyCbType = std::function<void(size_t size, size_t capacity)>;

        SerialProto();
        ~SerialProto();

        int connect(std::string portName, int baudrate);
        bool isConnected();

        /* Blocking */
        int sendCommand(const Command& cmd);

        /* Return the latest polled state */
        State getState();

        /* Controller command buffer utilization */
        float getBufStatus();

        /*
         * Only a single callback may be registered. The callback will be called
         * from the polling thread. notifyThreshold is the buffer utilization
         * factor on which the notification callback is called.
         */
        void registerBufNotifyCb(float notifyThreshold, BufNotifyCbType cb);

    private:
        std::mutex              _portLock;
#ifndef WITHOUT_LIBSERIALPORT
        sp_port*                _port;
#endif
        int                     _portTimeout;
        std::atomic<bool>       _connected;
        std::atomic<bool>       _pollThreadRunning;
        std::thread             _pollThread;
        std::mutex              _stateLock;
        State                   _state;
        float                   _bufNotifyThreshold;
        BufNotifyCbType         _bufNotifyCb;
        std::mutex              _bufStatusLock;
        uint32_t                _bufUsed;
        uint32_t                _bufCapa;
        const char*             _stateMsgFmt;
        const char*             _bufStatusRespFmt;

        int _doTransaction(const std::string& msg, std::string& resp);
        int _updateState();
        int _updateBufStatus();
        void _pollThreadFunc();
    };

}

#endif
