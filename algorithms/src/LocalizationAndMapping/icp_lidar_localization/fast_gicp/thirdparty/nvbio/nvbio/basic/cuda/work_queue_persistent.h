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
#include <thrust/device_vector.h>

namespace nvbio {
namespace cuda {

///@addtogroup WorkQueue
///@{

// queue tags
struct PersistentWarpsQueueTag {};
struct PersistentThreadsQueueTag {};

/// Implements a WorkQueue using persistent warps to fetch more work at a warp's granularity.
/// with dynamic work assignment (see \ref work_queue_page).
/// Each work-unit is assigned to a single thread, which runs until completion of all its continuations.
/// Useful if some warps finish much earlier than others, but all threads within a warp execute
/// roughly the same amount of work. Potentially destroys intra-CTA memory coherence, although
/// it maintains any warp-level coherence in the input stream.
/// Very low continuation overhead.
///
/// see \ref WorkQueue
///
template <
    typename WorkUnitT,
    uint32   BLOCKDIM>
struct WorkQueue<
    PersistentWarpsQueueTag,
    WorkUnitT,
    BLOCKDIM>
{
    typedef WorkUnitT   WorkUnit;

    /// constructor
    ///
    WorkQueue() : m_capacity(uint32(-1)) {}

    /// set queue capacity
    ///
    void set_capacity(const uint32 capacity) { m_capacity = capacity; }

    /// consume a stream of work units
    ///
    template <typename WorkStream>
    void consume(const WorkStream stream, WorkQueueStats* stats = NULL) { consume( stream, DefaultMover(), stats ); }

    /// consume a stream of work units
    ///
    template <typename WorkStream, typename WorkMover>
    void consume(const WorkStream stream, const WorkMover mover, WorkQueueStats* stats = NULL);

private:
    thrust::device_vector<uint32>   m_pool;
    uint32                          m_capacity;
};

/// Implements a WorkQueue using persistent warps to fetch more work at a warp's granularity.
/// with dynamic work assignment.
/// Each work-unit is assigned to a single thread, which runs until completion of all its continuations.
/// Useful if the number of continuations is fairly random.
/// Potentially destroys intra-CTA memory coherence.
/// Very low continuation overhead.
///
/// The user of this class have to specify a WorkStream class responsible for feeding work
/// to the queue in the shape of a subclass WorkStream::WorkUnit.
/// The latter is responsible for specifying the data and execution of each unit.
/// WorkStream has to implement two methods:
///
///    uint32 size() const
///    void   get(const uint32 i, WorkUnit* unit, const uint2 queue_slot)
///
/// When the method WorkQueue::consume( stream ) is called, the queue will launch a kernel
/// to consume all WorkUnit's in the stream.
/// WorkUnit has to implement a single method:
///
///    bool WorkUnit::run(const WorkStream& context)
///
/// which should run the associated work and indicate whether the unit has finished execution,
/// or whether it has produced a continuation (stored in the WorkUnit itself), that has to be
/// run further. The WorkQueue will automatically queue the continuation for later execution.
///
/// Optionally, the class can also be passed a WorkMover which is responsible for moving
/// additional data attached to any WorkUnit. This must implement a method:
///
///    void move(
///        const WorkStream& stream,
///        const uint2 src_slot, WorkUnit* src_unit,
///        const uint2 dst_slot, WorkUnit* dst_unit) const;
///
template <
    typename WorkUnitT,
    uint32   BLOCKDIM>
struct WorkQueue<
    PersistentThreadsQueueTag,
    WorkUnitT,
    BLOCKDIM>
{
    typedef WorkUnitT   WorkUnit;

    /// constructor
    ///
    WorkQueue() : m_capacity(uint32(-1)), m_min_utilization(0.75f) {}

    /// set queue capacity
    ///
    void set_capacity(const uint32 capacity) { m_capacity = capacity; }

    /// set utilization threshold
    ///
    void set_min_utilization(const float min_utilization) { m_min_utilization = min_utilization; }

    /// consume a stream of work units
    ///
    template <typename WorkStream>
    void consume(const WorkStream stream, WorkQueueStats* stats = NULL) { consume( stream, DefaultMover(), stats ); }

    /// consume a stream of work units
    ///
    template <typename WorkStream, typename WorkMover>
    void consume(const WorkStream stream, const WorkMover mover, WorkQueueStats* stats = NULL);

private:
    thrust::device_vector<uint32> m_pool;
    uint32                        m_capacity;
    float                         m_min_utilization;
};

} // namespace cuda
} // namespace nvbio

#include <nvbio/basic/cuda/work_queue_persistent_inl.h>
