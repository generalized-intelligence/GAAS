/*
 * nvbio
 * Copyright (c) 2011-2014, NVIDIA CORPORATION. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of the NVIDIA CORPORATION nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NVIDIA CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <nvbio/basic/types.h>
#include <nvbio/basic/numbers.h>
#include <nvbio/basic/atomics.h>
#include <nvbio/basic/shared_pointer.h>
#include <queue>

namespace nvbio {

/// \page threads_page Threads
///
/// This module implements basic host thread, synchronization and work-queue objects:
///
/// - Thread
/// - Mutex
/// - ScopedLock
/// - WorkQueue
/// - Pipeline
///

///@addtogroup Basic
///@{

///@defgroup Threads
/// This module implements basic host thread and work-queue objects.
///@{

uint32 num_physical_cores();
uint32 num_logical_cores();

class ThreadBase
{
public:
     ThreadBase();
    ~ThreadBase();

    void   set_id(const uint32 id) { m_id = id; }
    uint32 get_id() const { return m_id; }

    /// create the thread
    void create(void* (*func)(void*), void* arg);

    /// join the thread
    void join();

private:
    struct Impl;

    uint32                            m_id;
    SharedPointer<Impl, AtomicInt32>  m_impl;
};

/// A thread class, which is meant to be derived from to construct user-defined threads
///
/// \tparam DerivedThreadType     the derived thread class, which must implement a single method:
/// \code
/// // run the user-defined thread
/// void run();
/// \endcode
///
/// e.g.
/// \code
/// struct MyThread : public Thread<MyThread>
/// {
///     void run()
///     {
///        // do something
///        ...
///     }
/// }
/// \endcode
///
template <typename DerivedThreadType>
class Thread : public ThreadBase
{
public:
    /// create the thread
    void create() { ThreadBase::create( DerivedThreadType::execute, static_cast<DerivedThreadType*>(this) ); }

    /// join the thread
    void join() { ThreadBase::join(); }

private:
    /// execute
    static void* execute(void* arg)
    {
        DerivedThreadType* data = reinterpret_cast<DerivedThreadType*>( arg );
        data->run();
        return NULL;
    }
};

/// A mutex class, to be used to provide mutual exclusion of multi-thread execution
///
/// \code
/// struct MyThreadSafeClass
/// {
///     // multi-thread safe section
///     void foo()
///     {
///        bar();
///
///        // enter a critical section
///        m_mutex.lock();
///        ... // do something
///        m_mutex.unlock();
///     }
/// private:
///     Mutex m_mutex;
/// };
/// \endcode
///
class Mutex
{
public:
     Mutex();
    ~Mutex();

    void lock();
    void unlock();

private:
    struct Impl;

    SharedPointer<Impl, AtomicInt32>  m_impl;
};

/// A scoped lock, to be used to protect a code section enclosed within a scope.
/// e.g.
///
/// \code
/// struct MyThreadSafeClass
/// {
///     // multi-thread safe section
///     void foo()
///     {
///        bar();
///
///        // enter a critical section
///        {
///            ScopedLock(&m_mutex);
///            ... // do something
///        }
///     }
/// private:
///     Mutex m_mutex;
/// };
/// \endcode
///
///
class ScopedLock
{
public:
     ScopedLock(Mutex* mutex) : m_mutex( mutex ) { m_mutex->lock(); }
    ~ScopedLock() { m_mutex->unlock(); }

private:
    Mutex* m_mutex;
};

/// Work queue class
template <typename WorkItemT, typename ProgressCallbackT>
class WorkQueue
{
public:
    typedef WorkItemT           WorkItem;
    typedef ProgressCallbackT   ProgressCallback;

    /// empty constructor
    WorkQueue() : m_callback(), m_size(0u) {}

    /// push a work item in the queue
    void push(const WorkItem work) { m_queue.push( work ); m_size++; }

    /// push a work item in the queue
    void locked_push(const WorkItem work)
    {
        ScopedLock block( &m_lock );
        m_queue.push( work ); m_size++;
    }

    /// pop the next work item from the queue
    bool pop(WorkItem& work)
    {
        ScopedLock block( &m_lock );
        if (m_queue.empty())
            return false;

        work = m_queue.front();
        m_queue.pop();

        m_callback( m_size -( uint32)m_queue.size() - 1u, m_size );
        return true;
    }

    /// set a callback
    void set_callback(const ProgressCallback callback) { m_callback = callback; }

private:
    ProgressCallback      m_callback;
    std::queue<WorkItem>  m_queue;
    Mutex                 m_lock;
    uint32                m_size;
};

/// return a number close to batch_size that achieves best threading balance
inline uint32 balance_batch_size(uint32 batch_size, uint32 total_count, uint32 thread_count)
{
    // How many batches we'd get with the proposed batch_size
    const uint32 batch_count = util::divide_ri(total_count, batch_size);
    // How many rounds we'd need for those batches
    const uint32 rounds      = util::divide_ri(batch_count, thread_count);
    // Might as well assume all threads should work, and see how many batches
    // they would consume
    const uint32 bal_batches = rounds * thread_count;
    // So that the batch size that will attain it, is computed as follows
    return util::divide_ri(total_count, bal_batches);
}

void yield();

///@} Threads
///@} Basic

} // namespace nvbio
