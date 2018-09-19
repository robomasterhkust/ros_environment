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

/**
  * @brief 
  * a storage class for storing data buffers by their pointers
  * @tparam T 
 */
template <class T>
class ConcurrentQueue
{
  public:
    ConcurrentQueue(const int &_bufferSize = 2)
        : bufferSize((_bufferSize > 1) ? _bufferSize : 2), // at least 2 to work
          toWritePos(0),
          toReadPos(0)
    {
        buffer = new T *[bufferSize];
        for (int i = 0; i < bufferSize; i++)
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

    /**
    * @brief 
    * enqueue by a pointer to the date, no data copying is occuring here,
    * do not call the delete function after using this, please, the memory will be
    * handled by this thing or by the function which called the dequeue funtion to retrieve the pointer 
    * @param in 
    */
    void enqueue(T *const &in)
    {
        lock_guard<mutex> block(blocker);

        delete buffer[toWritePos];
        buffer[toWritePos] = in;

        toWritePos = (toWritePos + 1) % bufferSize;
        if (toWritePos == toReadPos)
            toReadPos = ((toReadPos + 1) >= bufferSize) ? (0) : (toReadPos + 1);
        return;
    };

    /**
      * @brief 
      * pop a data pointer from the queue
      * @return true data available and is sucessfully poped
      * @return false no data is available
     */
    bool dequeue(T *&out)
    {
        lock_guard<mutex> block(blocker);
        bool result;
        if (result = (toWritePos != toReadPos))
        {
            out = buffer[toReadPos];
            buffer[toReadPos] = NULL;
            toReadPos = ((toReadPos + 1) >= bufferSize) ? (0) : (toReadPos + 1);
        }
        return result;
    };

    /**
     * @brief direcetly obatin the pointer to the queued object without poping it, lifetime not guranteeded, use shared pointer here?
     * 
     * 
     * @param out 
     * @return true 
     * @return false 
     */
    bool peek(T *&out)
    {
        unsigned int loopCount = 5;
        while (toWritePos == toReadPos && loopCount > 0)
        {
            blocker.lock();
            if (toWritePos != toReadPos)
            {
                out = buffer[toReadPos];
                return true;
            }
            blocker.unlock();
            loopCount--;
        }
        cout << "loopcount reached \n";
        return false;
    }

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
    volatile unsigned int toWritePos;
    volatile unsigned int toReadPos;
    const int bufferSize;
    T **buffer;
};