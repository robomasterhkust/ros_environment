#pragma once
#include <atomic>
#include <thread>
#include <assert.h>
#include <iostream>
#include <cstring>
#include <mutex>

using namespace std;

//this may not be same as other library's concurretn queue, but might best fit the need fo cv
//
//behavior:
//  -like a circular queue of fixed size, but enqueuing is always possible
//  -buffer is full, enqueue will also push dequeue position forawrd
//  -in this way, the data obtained by dequeue function will at most be [buffersize-1]'th data
//  the most recent enqueue

//only one thread use enqueue, one dequeue, please

//TODO: store pointers to data instead of the data itself and enforce memory management here?
//TODO: function to check free space, compute only if space available to harvest most compute efficiency

template <class T>
class ConcurrentQueue
{
  public:
    ConcurrentQueue(const int &_bufferSize = 2)
        : bufferSize((_bufferSize > 1) ? _bufferSize : 2), // at least 2 to work
          toWritePos(0),
          toReadPos(0),
          buffer(new T *[_bufferSize])
    {
        for (int i = 0; i < _bufferSize; i++)
        {
            buffer[i] = NULL;
        }
    };

    ~ConcurrentQueue()
    {
        for (int i = 0; i < bufferSize; i++)
            delete buffer[i];
        delete[] buffer;
    };

    void enqueue(T *const &in)
    {
        lock_guard<mutex> block(blocker);

        delete buffer[toWritePos];
        buffer[toWritePos] = in;

        toWritePos = (toWritePos + 1) % bufferSize;
        if (toWritePos == toReadPos)
            toReadPos = (toReadPos + 1) % bufferSize;
    };

    //pop a datum if available and return true, false otherwise
    //the pointer will not be handled once its poped
    bool dequeue(T *&out)
    {
        lock_guard<mutex> block(blocker);
        bool result;
        if (result = (toWritePos != toReadPos))
        {
            out = buffer[toReadPos];
            buffer[toReadPos] = NULL;
            toReadPos = (toReadPos + 1) % bufferSize;
        }
        return result;
    };

    void reset()
    {
        std::lock_guard<std::mutex> block(blocker);
        for (int i = 0; i < bufferSize; i++)
            delete buffer[i];
        delete[] buffer;
        buffer = new T[bufferSize];
        toWritePos = 0;
        toReadPos = 0;
    }

    int count()
    {
        std::lock_guard<std::mutex> block(blocker);
        int temp = (toWritePos - toReadPos);
        if (temp < 0)
            temp += bufferSize;
        return temp;
    };

    int getFreeSpace()
    {
        std::lock_guard<std::mutex> block(blocker);
        int temp = (toWritePos - toReadPos);
        if (temp < 0)
            temp += bufferSize;
        return (bufferSize - temp);
    }

    int getBufferSize()
    {
        return bufferSize;
    };

    // void closeWriting()
    // {
    //     inProduction = false;
    // };

    // atomic<bool> inProduction;

  private:
    //dequeue can only access startPos, dequeue only can access toWritePos
    //these Pos integers are incremented only if the value after the increment isn't the other
    mutex blocker;
    atomic<int> toWritePos, toReadPos;
    const int bufferSize;
    T **const buffer;
};