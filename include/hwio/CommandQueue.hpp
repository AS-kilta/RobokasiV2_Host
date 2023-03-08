#ifndef ROBOKASIV2_HOST_COMMANDQUEUE_HPP
#define ROBOKASIV2_HOST_COMMANDQUEUE_HPP


#include "hwio/SerialProto.hpp"

#include <deque>
#include <mutex>
#include <vector>


namespace hwio {

    class CommandQueue {
    public:
        CommandQueue(SerialProto& serial);
        ~CommandQueue() = default;
        void bufferNotify(size_t size, size_t capacity);
        void addCommands(std::vector<Command>& cmds);
        void addCommand(const Command& cmds);
        void clearQueue();

        size_t size();
        unsigned duration();
    private:
        SerialProto& _serial;
        /* _queue is accessed from another thread via bufferNotify() */
        std::mutex _queueLock;
        std::deque<Command> _queue;
    };

}

#endif
