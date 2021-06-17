#ifndef CALLBACK_BUFFER_BLOCK_H
#define CALLBACK_BUFFER_BLOCK_H


//#include <queue>
#include <vector>
#include <deque>
#include <iostream>
#include <glog/logging.h>
using namespace std;
template <typename T>
class CallbackBufferBlock
{
public:
    CallbackBufferBlock<T>()
    {
	    ;
    }
    void onCallbackBlock(const T& msg);
    double queryLastMessageTime();
    T getLastMessage();
    void clear()
    {
        this->msgBuffer.clear();
    }
    inline int size()
    {
        return this->msgBuffer.size();
    }
    std::vector<T> getCopyVec()
    {
        std::vector<T> vec;
        int size = this->msgBuffer.size();
        for(int i=0;i<size;i++)
        {
            vec.push_back(this->msgBuffer.at(i));
        }
        return vec;
    }
    T operator[](int index)
    {
        return this->msgBuffer[index];
    }
    T at(int index)
    {
        if(index >= msgBuffer.size())
        {
            LOG(ERROR) <<"ERROR:dereference index overflow!"<<endl;
            LOG(ERROR) <<"index: "<<index<<",size:"<<msgBuffer.size()<<endl;
        }
        return this->msgBuffer[index];
    }
    /*inline std::deque<T>& getBuffer()//NOT recommended!
    {
        return this->msgBuffer;
    }*/
private:
    //std::queue<T> msgBuffer;
    std::deque<T> msgBuffer;
    //const int buffer_size = 100;
};
template <typename T>
void CallbackBufferBlock<T>::onCallbackBlock(const T& msg)
{
    //access: .front(),.back().
    //push_front,push_back();
    //pop_front,pop_back();
    //at(),operator []
    this->msgBuffer.push_back(msg); //msgBuffer.front() oldest.msgBuffer.end() latest.
    //if (this->msgBuffer.size()>=this->buffer_size)
    //{
    //    this->msgBuffer.pop_back();
    //}
}
template <typename T>
double CallbackBufferBlock<T>::queryLastMessageTime()
{
    double ret_time = msgBuffer.at(this->msgBuffer.size()-1).header.stamp.toSec();
    cout<<"MessageTime:"<<ret_time<<endl;
    return ret_time;
}
template <typename T>
T CallbackBufferBlock<T>::getLastMessage()
{
    if (this->msgBuffer.size() > 0)
    {
        return this->msgBuffer.end();
    }
}

#endif




