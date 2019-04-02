#ifndef CALLBACK_BUFFER_BLOCK_H
#define CALLBACK_BUFFER_BLOCK_H


#include <queue>

template typename<T>
class CallbackBufferBlock
{
public:
    CallbackBlock();
    void onCallbackBlock(const T& msg);
    double queryLastMessageTime();
    T getLastMessage();
    inline int size()
    {
        return this->msgBuffer.size();
    }
private:
    std::queue<T> msgBuffer;
    const int buffer_size = 100;
};

CallbackBlock<T>::onCallbackBlock(const T& msg)
{
    this->msgBuffer.push(msg); //msgBuffer.front() oldest.msgBuffer.end() latest.
    if (this->msgBuffer.size()>=this->buffer_size)
    {
        this->msgBuffer.pop();
    }
}
double CallbackBlock<T>::queryLastMessageTime()
{
    return this->msgBuffer.back().header.stamp.toSec();
}
T CallbackBufferBlock<T>::getLastMessage()
{
    if (this->msgBuffer.size() > 0)
    {
        return this->msgBuffer.end();
    }
}

#endif




