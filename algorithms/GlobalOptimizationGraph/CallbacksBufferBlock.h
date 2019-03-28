#ifndef CALLBACK_BUFFER_BLOCK_H
#define CALLBACK_BUFFER_BLOCK_H

template typename<T>
class CallbackBufferBlock
{
public:
    CallbackBlock();
    void onCallbackBlock(const T& msg);
    double queryLastMessageTime();
private:
    std::vector<T> msgBuffer;
    int buffer_size;
};

CallbackBlock<T>::onCallbackBlock()
{
    
}
CallbackBlock<T>::queryLastMessageTime()
{

}

#endif
