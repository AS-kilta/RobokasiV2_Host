#include "hwio/CommandQueue.hpp"

#include <mutex>


using namespace hwio;


CommandQueue::CommandQueue(SerialProto& serial) :
    _serial(serial)
{
}

void CommandQueue::bufferNotify(size_t size, size_t capacity)
{
    std::lock_guard<std::mutex> guard(_queueLock);
    for (size_t i = size; i < capacity && !_queue.empty(); ++i) {
        if (!_serial.isConnected()) {
            _queue.clear();
            return;
        }
        _serial.sendCommand(_queue.front());
        _queue.pop_front();
   }
}

void CommandQueue::addCommands(std::vector<Command>& cmds)
{
    std::lock_guard<std::mutex> guard(_queueLock);
    std::move(begin(cmds), end(cmds), back_inserter(_queue));
}

void CommandQueue::addCommand(const Command& cmd)
{
    std::lock_guard<std::mutex> guard(_queueLock);
    _queue.push_back(cmd);
}

size_t CommandQueue::size()
{
    std::lock_guard<std::mutex> guard(_queueLock);
    return _queue.size();
}

unsigned CommandQueue::duration()
{
    std::lock_guard<std::mutex> guard(_queueLock);

    unsigned t = 0;

    for (const Command& cmd : _queue)
        t += cmd.dt;

    return t;
}
