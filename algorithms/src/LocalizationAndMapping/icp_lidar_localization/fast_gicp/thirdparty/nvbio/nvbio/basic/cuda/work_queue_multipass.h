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

#include <nvbio/basic/cuda/work_queue.h>
#include <nvbio/basic/types.h>
#include <nvbio/basic/numbers.h>
#include <thrust/device_vector.h>

namespace nvbio {
namespace cuda {

///@addtogroup WorkQueue
///@{

struct MultiPassQueueTag {};

/// Implements a multi-pass WorkQueue that uses thrust::copy_if to compact
/// continuations between kernel launches (see \ref work_queue_page).
/// Mantains input ordering in the thread assignment, hence mantaining memory access
/// coherence, but has potentially much higher overhead than the Ordered queue if the
/// queue capacity is low.
/// High continuation overhead at low queue capacities. At high queue capacities
/// (e.g. millions of elements), the kernel launch overheads are amortized.
///
/// see \ref WorkQueue
///
template <
    typename WorkUnitT,
    uint32   BLOCKDIM>
struct WorkQueue<
    MultiPassQueueTag,
    WorkUnitT,
    BLOCKDIM>
{
    typedef WorkUnitT   WorkUnit;

    /// constructor
    ///
    WorkQueue() : m_capacity(32*1024), m_separate_loads(false) {}

    /// set queue capacity
    ///
    void set_capacity(const uint32 capacity) { m_capacity = capacity; }

    /// enable separate loads
    ///
    void set_separate_loads(const bool flag) { m_separate_loads = flag; }

    /// consume a stream of work units
    ///
    template <typename WorkStream>
    void consume(const WorkStream stream, WorkQueueStats* stats = NULL) { consume( stream, DefaultMover(), stats ); }

    /// consume a stream of work units
    ///
    template <typename WorkStream, typename WorkMover>
    void consume(const WorkStream stream, const WorkMover mover, WorkQueueStats* stats = NULL);

    struct Context
    {
        WorkUnit*  m_work_queue;
        uint32*    m_continuations;
    };

private:
    /// get a context
    ///
    Context get_context()
    {
        Context context;
        context.m_work_queue      = thrust::raw_pointer_cast( &m_work_queue.front() );
        context.m_continuations   = thrust::raw_pointer_cast( &m_continuations.front() );
        return context;
    }

    uint32                          m_capacity;
    bool                            m_separate_loads;
    thrust::device_vector<WorkUnit> m_work_queue;
    thrust::device_vector<uint32>   m_continuations;
};

///@} // WorkQueue

} // namespace cuda
} // namespace nvbio

#include <nvbio/basic/cuda/work_queue_multipass_inl.h>
