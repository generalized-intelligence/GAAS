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

///\page work_queue_page Work-Queues
///
///\par
/// This module contains a series of classes that implement a variety of CUDA work-queues.
/// A work-queue is an object which allows to execute (or consume) a stream of work items
/// according to different schedulers.
/// Notably, there's two distinguishing features:
/// - <em><b>continuations:</b></em>
///   work items can produce <i>continuations</i>: breaking up long computations in shorter
///   pieces and using smart schedulers might allow to get better utilization when the execution
///   time of individual work items is highly non-uniform;
/// - <em><b>capacity constraints:</b></em>
///   queues can be assigned a <i>maximum capacity</i>, which can be used to control the amount
///   of resources consumed to execute the work items in parallel (e.g. if each work item needs
///   1MB of temporary storage, on a 4GB GPU one might only execute 4k items in parallel)
///
///\section Work-Streams
///\par
/// The user of these classes have to specify a WorkStream class responsible for feeding work
/// to the queue in the shape of a subclass WorkStream::WorkUnit.
/// The latter is responsible for specifying the data and execution of each unit.
/// WorkStream has to implement the following interface:
///
/// \code
/// interface WorkStream
/// {
///    uint32 size() const
///    void   get(const uint32 i, WorkUnit* unit, const uint2 queue_slot)
/// }
/// \endcode
///\par
/// queue_slot specifies a (queue position, queue id) pair, assuming there can be one
/// input and one output continuation queue which are swapped among iterations.
/// Knowing the queue slot can be useful to bind external data to a work-unit.
///\par
/// When the method WorkQueue::consume( stream ) is called, the queue will launch a kernel
/// to consume all WorkUnit's in the stream.
/// WorkUnit has to implement a single method:
///
/// \code
///    bool WorkUnit::run(const WorkStream& context)
/// \endcode
///\par
/// which should run the associated work and indicate whether the unit has finished execution,
/// or whether it has produced a continuation (stored in the WorkUnit itself), that has to be
/// run further. The WorkQueue will automatically queue the continuation for later execution.
///\par
/// Optionally, the class can also be passed a WorkMover which is responsible for moving
/// external data attached to any WorkUnit when its continuation gets assigned a new execution
/// slot. This must implement a method:
///
/// \code
///    void move(
///        const WorkStream& stream,
///        const uint2 src_slot, WorkUnit* src_unit,
///        const uint2 dst_slot, WorkUnit* dst_unit) const;
/// \endcode
///
///\section WorkQueueExampleSection Example
///
/// \code
///    struct MyWorkStream;
///
///    // A work unit returning a continuation for odd indices.
///    //
///    struct MyWorkUnit
///    {
///        // construct this work unit
///        __device__ MyWorkUnit(const uint32 _i) : i(_i) {}
///
///        // run this work unit
///        __device__ bool run(MyWorkStream&);
///
///    private:
///        uint32 i;
///    }
///
///    // A stream of work units
///    //
///    struct MyWorkStream
///    {
///        // construct this work stream
///        MyWorkStream(const _size) : m_size(_size) {}
///
///        // return the size of the stream
///        __host__ __device__ uint32 size() const { return m_size; }
///
///        // get a work unit, assigned to a given execution slot
///        __device__ void get(const uint32 i, MyWorkUnit* unit, const uint2 execution_slot) const;
///
///    private:
///        uint32 m_size;
///    }
///
///    // get a work unit
///    __device__ void MyWorkStream::get(const uint32 i, MyWorkUnit* unit, const uint2 execution_slot) const { *unit = MyWorkUnit(i); }
///
///    // run the work unit
///    __device__ bool MyWorkUnit::run(MyWorkStream&) { if (i&1) { i/=2; return true; return false; }
///
///    void test()
///    {
///        // instantiate a work stream
///        MyWorkStream work_stream( 1024*1024 );
///
///        // instantiated a work queue
///        cuda::WorkQueue<cuda::InplaceQueueTag,MyWorkUnit> work_queue;
///
///        // consume all work in the work stream
///        work_queue.consume( work_stream );
///    }
/// \endcode
///
///\section WorkQueueSchedulersSection Work-Queue Schedulers
///
/// The WorkQueue class is parameterized by a template tag parameter specifying the scheduler.
/// The available schedulers are:
///
/// - InplaceQueueTag               (see WorkQueue)
/// - MultiPassQueueTag             (see WorkQueue<MultiPassQueueTag,WorkUnitT,BLOCKDIM>)
/// - PersistentWarpsQueueTag       (see WorkQueue<PersistentWarpsQueueTag,WorkUnitT,BLOCKDIM>)
/// - PersistentThreadsQueueTag     (see WorkQueue<PersistentThreadsQueueTag,WorkUnitT,BLOCKDIM>)
/// - OrderedQueueTag               (see WorkQueue<OrderedQueueTag,WorkUnitT,BLOCKDIM>)
///

///
///@defgroup WorkQueue Work-Queues
///
/// This module contains a series of classes that implement a variety of CUDA work-queues.
/// A work-queue is an object which allows to execute (or consume) a stream of work items
/// according to different schedulers.
/// See \ref work_queue_page for more details.
///@{
///

// default tag
struct InplaceQueueTag {};

// a simple class to keep track of utilization stats
struct WorkQueueStats;
struct WorkQueueStatsView;

// return a device-side view of a stats object
inline WorkQueueStatsView view(WorkQueueStats* stats);

///
/// a work-queue stats event
///
enum WorkQueueStatsEvent {
    STREAM_EVENT = 0,
    RUN_EVENT    = 1,
};

///
/// a device-side view
///
struct WorkQueueStatsView
{
    /// default constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    WorkQueueStatsView() : active_lanes(NULL), issued_warps(NULL), iterations(NULL) {}

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    WorkQueueStatsView(uint64* _active_lanes, uint64* _issued_warps, uint64* _iterations) :
        active_lanes(_active_lanes), issued_warps(_issued_warps), iterations(_iterations) {}

    /// sample utilization
    ///
    NVBIO_FORCEINLINE NVBIO_DEVICE
    void sample(const WorkQueueStatsEvent type);

    /// sample avg/max iterations
    ///
    NVBIO_FORCEINLINE NVBIO_DEVICE
    void sample_iterations(const uint32 i);

    /// is the view valid
    ///
    NVBIO_FORCEINLINE NVBIO_DEVICE
    bool valid() const { return active_lanes != NULL; }

    uint64* active_lanes;
    uint64* issued_warps;
    uint64* iterations;
};

///
/// generic implementation of a move function for WorkUnit's
///
/// This class is responsible for moving any external payload bound to a work-unit,
/// and gets invoked when the work-queue changes the execution slot of a continuation
/// relative to its parent.
///
struct DefaultMover
{
    template <typename WorkStreamT, typename WorkUnitT>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void move(
        const WorkStreamT& stream,
        const uint2 src_slot, WorkUnitT* src_unit,
        const uint2 dst_slot, WorkUnitT* dst_unit) const
    {
        // call WorkUnitT's copy operator and call it a day
        *dst_unit = *src_unit;
    }
};

/// Implements a parallel work queue interface (see \ref work_queue_page).
/// This default implementation employs a single kernel launch with static thread assignment
/// and in-place execution of a work-unit and all its continuations.
/// It has very low continuation overhead, but it might suffer from poor SIMT utilization
/// when work-units have wildly varying number of continuations.
///
template <
    typename PolicyTag,
    typename WorkUnitT,
    uint32   BLOCKDIM>
struct WorkQueue
{
    typedef WorkUnitT   WorkUnit;

    /// constructor
    ///
    WorkQueue() {}

    /// set queue capacity.
    ///
    /// In this context, the queue capacity is the maximum amount of persistent threads
    /// used to run the stream.
    /// Limiting it might be useful when the work-units reference some limited external
    /// temporary storage.
    ///
    void set_capacity(const uint32 capacity) {}

    /// consume a stream of work units
    ///
    template <typename WorkStream>
    void consume(const WorkStream stream, WorkQueueStats* stats = NULL) { consume( stream, DefaultMover(), stats ); }

    /// consume a stream of work units
    ///
    template <typename WorkStream, typename WorkMover>
    void consume(const WorkStream stream, const WorkMover mover, WorkQueueStats* stats = NULL);
};

///
/// a simple class to keep track of utilization stats
///
struct WorkQueueStats
{
    typedef WorkQueueStatsView View;

    /// default constructor
    ///
    WorkQueueStats() : counters(7,0) {}

    /// clear all stats
    ///
    void clear() { thrust::fill( counters.begin(), counters.end(), uint64(0u) ); }

    /// return a device-side view
    ///
    View view()
    {
        return View(
            thrust::raw_pointer_cast( &counters.front() ),
            thrust::raw_pointer_cast( &counters.front() ) + 2u,
            thrust::raw_pointer_cast( &counters.front() ) + 4u );
    }

    /// return measured utilization
    ///
    float utilization(const WorkQueueStatsEvent type) const { return counters[2 + type] ? float(counters[0 + type])/float(counters[2 + type]*cuda::Arch::WARP_SIZE) : 1.0f; }

    /// return measured iterations
    ///
    float avg_iterations() const { return counters[6] ? float(counters[4])/float(counters[6]) : 0.0f; }

    /// return measured iterations
    ///
    float max_iterations() const { return float(counters[5]); }

private:
    thrust::device_vector<uint64>   counters;
};

/// return a device-side view
///
inline WorkQueueStatsView view(WorkQueueStats* stats) { return stats ? stats->view() : WorkQueueStatsView(); }

///@} // WorkQueue

} // namespace cuda
} // namespace nvbio

#include <nvbio/basic/cuda/work_queue_inl.h>
#include <nvbio/basic/cuda/work_queue_persistent.h>
#include <nvbio/basic/cuda/work_queue_ordered.h>
#include <nvbio/basic/cuda/work_queue_multipass.h>
