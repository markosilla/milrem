#pragma once

#include <vector>
#include <iostream>
#include <stdexcept>
#include <mutex>
#include <condition_variable>

// https://en.wikipedia.org/wiki/Circular_buffer
// https://fluentprogrammer.com/beautiful-code-8-conditional-variable-multi-threading-in-cpp/

namespace common
{
    template <typename T>
    class RingBuffer
    {
    public:
        RingBuffer(int capacity);

        void put(T &element);
        T get();
        bool empty() const;
        bool full() const;

    private:
        std::vector<T> buffer;
        int writeIndex;
        int readIndex;
        int capacity;
        std::mutex mutex;
        std::condition_variable dataAvailable;
        
        bool is_empty_unsafe() const;
        bool is_full_unsafe() const;
    };

    template <typename T>
    RingBuffer<T>::RingBuffer(int capacity) : buffer(capacity), writeIndex(0), readIndex(0), capacity(capacity) {}

    template <typename T>
    void RingBuffer<T>::put(T &element)
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (is_full_unsafe()) {
            throw std::runtime_error("Buffer is full");
        }
        buffer[writeIndex] = element;
        writeIndex = (writeIndex + 1) % capacity;
        dataAvailable.notify_one();
    }

    template <typename T>
    T RingBuffer<T>::get()
    {
        std::unique_lock<std::mutex> lock(mutex);
        dataAvailable.wait(lock, [this] { return !is_empty_unsafe(); });
        auto val = buffer[readIndex];
        readIndex = (readIndex + 1) % capacity;
        return val;
    }

    template <typename T>
    bool RingBuffer<T>::full() const
    {
        std::lock_guard<std::mutex> lock(mutex);
        return is_full_unsafe();
    }

    template <typename T>
    bool RingBuffer<T>::empty() const
    {
        std::lock_guard<std::mutex> lock(mutex);
        return is_empty_unsafe();
    }
    
    template <typename T>
    bool RingBuffer<T>::is_empty_unsafe() const
    {
        return readIndex == writeIndex;
    }

    template <typename T>
    bool RingBuffer<T>::is_full_unsafe() const
    {
        return (writeIndex + 1) % capacity == readIndex;
    }
} // namespace common