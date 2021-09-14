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
#include <nvbio/basic/cuda/condition.h>
#include <nvbio/basic/cuda/syncblocks.h>
#include <thrust/device_vector.h>

namespace nvbio {
namespace cuda {

///@addtogroup WorkQueue
///@{

struct OrderedQueueTag {};

/// Implements a self-compacting parallel WorkQueue, using a single-kernel launch that
/// compacts continuations maintaining while input ordering in the thread assignment (see \ref work_queue_page).
/// Useful to maintain memory access coherence.
/// Relatively high continuation overhead, but much lower than the MultiPass queue
/// if the queue capacity is low.
///
/// see \ref WorkQueue
///
template <
    typename WorkUnitT,
    uint32   BLOCKDIM>
struct WorkQueue<
    OrderedQueueTag,
    WorkUnitT,
    BLOCKDIM>
{
    typedef WorkUnitT   WorkUnit;

    /// constructor
    ///
    WorkQueue() : m_capacity(32*1024) {}

    /// set queue capacity
    ///
    void set_capacity(const uint32 capacity) { m_capacity = capacity; }

    /// consume a stream of work units
    ///
    template <typename WorkStream, typename WorkMover>
    void consume(const WorkStream stream, const WorkMover mover);

    /// consume a stream of work units
    ///
    template <typename WorkStream>
    void consume(WorkStream stream) { consume( stream, DefaultMover() ); }

    struct Context
    {
                 WorkUnit*  m_work_queue;
        volatile uint32*    m_work_queue_size;
        volatile uint32*    m_partials;
        volatile uint32*    m_prefixes;
                 uint8*     m_continuations;
                 uint32*    m_source_ids;
        condition_set_view  m_conditions;
        syncblocks          m_syncblocks;
    };

private:
    /// get a context
    ///
    Context get_context()
    {
        Context context;
        context.m_work_queue      = thrust::raw_pointer_cast( &m_work_queue.front() );
        context.m_work_queue_size = thrust::raw_pointer_cast( &m_work_queue_size.front() );
        context.m_partials        = thrust::raw_pointer_cast( &m_partials.front() );
        context.m_prefixes        = thrust::raw_pointer_cast( &m_prefixes.front() );
        context.m_continuations   = thrust::raw_pointer_cast( &m_continuations.front() );
        context.m_source_ids      = thrust::raw_pointer_cast( &m_source_ids.front() );
        context.m_conditions      = m_condition_set.get();
        context.m_syncblocks      = m_syncblocks.get();
        return context;
    }

    uint32                          m_capacity;
    thrust::device_vector<WorkUnit> m_work_queue;
    thrust::device_vector<uint32>   m_work_queue_size;
    thrust::device_vector<uint32>   m_partials;
    thrust::device_vector<uint32>   m_prefixes;
    thrust::device_vector<uint8>    m_continuations;
    thrust::device_vector<uint32>   m_source_ids;
    condition_set_storage           m_condition_set;
    syncblocks_storage              m_syncblocks;
};

///@} // WorkQueue

} // namespace cuda
} // namespace nvbio

#include <nvbio/basic/cuda/work_queue_ordered_inl.h>
