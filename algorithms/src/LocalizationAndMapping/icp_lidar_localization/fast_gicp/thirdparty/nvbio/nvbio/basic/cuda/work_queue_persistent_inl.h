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
#include <nvbio/basic/cuda/arch.h>
#include <cub/cub.cuh>
#include <thrust/copy.h>

namespace nvbio {
namespace cuda {

///@addtogroup WorkQueue
///@{

//
// -----------------------   PersistentWarpsQueueTag Implementation -------------------------------------
//

namespace wq {

///@addtogroup WorkQueueDetail
///@{

template <
    uint32   BLOCKDIM,
    typename WorkUnitT,
    typename WorkStreamT>
__global__
void persistent_warps_work_queue_kernel(uint32* pool, const WorkStreamT stream, WorkQueueStats::View stats)
{
    typedef WorkUnitT WorkUnit;

    // compute the global thread id
    const uint32 thread_id = threadIdx.x + blockIdx.x*BLOCKDIM;

    // place a work-unit in local memory
    WorkUnit unit;

    const uint32 NUM_WARPS = BLOCKDIM >> cuda::Arch::LOG_WARP_SIZE;

    // alloc one shared memory integer for each warp
    __shared__ volatile uint32 sm_broadcast[ NUM_WARPS ];

    const uint32 stream_size = stream.size();
    while (1)
    {
        // use the first lane of each warp to fetch a new warp's worth of work
        if (warp_tid() == 0)
            sm_broadcast[ warp_id() ] = atomicAdd( pool, cuda::Arch::WARP_SIZE );

        // broadcast the work packet to the entire warp
        const uint32 work_base_id = sm_broadcast[ warp_id() ];

        // retire this warp if done
        if (work_base_id >= stream_size)
            return;

        // get the index of the work unit for this thread
        const uint32 work_id = warp_tid() + work_base_id;

        if (work_id < stream_size)
        {
            // fetch the work unit
            stats.sample( STREAM_EVENT );
            stream.get( work_id, &unit, make_uint2( thread_id, 0u ) );

            // keep an iteration counter
            uint32 work_iter = 0;

            // run the unit until completion
            do { stats.sample( RUN_EVENT ); ++work_iter; } while (unit.run( stream ));

            // sample the number of iterations this unit has been running
            stats.sample_iterations( work_iter );
        }
    }
}

} // namespace wq

// consume a stream of work units
//
template <
    typename WorkUnitT,
    uint32   BLOCKDIM>
template <typename WorkStream, typename WorkMover>
void WorkQueue<PersistentWarpsQueueTag,WorkUnitT,BLOCKDIM>::consume(const WorkStream stream, const WorkMover, WorkQueueStats* stats)
{
    // compute the number of blocks we are going to launch
    const uint32 max_blocks = (uint32)cuda::max_active_blocks( wq::persistent_warps_work_queue_kernel<BLOCKDIM,WorkUnit,WorkStream>, BLOCKDIM, 0u );
    const uint32 n_blocks   = nvbio::max( nvbio::min( max_blocks, m_capacity / BLOCKDIM ), 1u );


    // resize and reset the work pool counter
    m_pool.resize(1);
    m_pool[0] = 0u;

    // launch the consuming kernel
    wq::persistent_warps_work_queue_kernel<BLOCKDIM,WorkUnit,WorkStream> <<<n_blocks,BLOCKDIM>>>( thrust::raw_pointer_cast( &m_pool.front() ), stream, view( stats ) );
}

//
// -----------------------   PersistentThreadsQueueTag Implementation ----------------------------------
//

namespace wq {

template <
    uint32   BLOCKDIM,
    typename WorkUnitT,
    typename WorkStreamT>
__global__
void
__launch_bounds__(BLOCKDIM, 6)
persistent_threads_work_queue_kernel(uint32* pool, const uint32 max_inactive_lanes, const WorkStreamT stream, WorkQueueStats::View stats)
{
    typedef WorkUnitT WorkUnit;

    // compute the global thread id
    const uint32 thread_id = threadIdx.x + blockIdx.x*BLOCKDIM;

    // place a work-unit in local memory
    WorkUnit unit;

    const uint32 NUM_WARPS = BLOCKDIM >> cuda::Arch::LOG_WARP_SIZE;

    // alloc one shared memory integer for each warp
    __shared__ volatile uint32 sm_broadcast[ NUM_WARPS ];

    const uint32 invalid_unit = uint32(-1);

    // start all threads with unassigned work units
    uint32 work_id   = invalid_unit;
    uint32 work_iter = 0u;
    bool   active    = true;

    const uint32 stream_size = stream.size();
    while (__any(active))
    {
        // check how many lanes need some new work to do
        const uint32 pop_mask  = __ballot( work_id == invalid_unit );
        const uint32 pop_count = __popc( pop_mask );

        // refill this warp only when utilization falls below a certain threshold
        if (pop_count > max_inactive_lanes)
        {
            // use the first lane of each warp to fetch a new warp's worth of work
            if (warp_tid() == 0)
                sm_broadcast[ warp_id() ] = atomicAdd( pool, pop_count );

            // broadcast the work packet to the entire warp
            const uint32 work_base_id = sm_broadcast[ warp_id() ];

            // let inactive lanes gather a new work unit
            if (work_id == invalid_unit)
            {
                // compute this lane's exclusive pop scan
                const uint32 pop_scan = __popc( pop_mask << (cuda::Arch::WARP_SIZE - warp_tid()) );

                // get the index of the work unit for this thread
                work_id   = pop_scan + work_base_id;
                work_iter = 0u;

                // retire this thread if done
                if (work_id < stream_size)
                {
                    // fetch the work unit
                    stream.get( work_id, &unit, make_uint2( thread_id, 0u ) );
                    stats.sample( STREAM_EVENT );
                }
                else // signal the main loop that, if it was for us, we could stop
                {
                    active  = false;
                    work_id = invalid_unit;
                }
            }
        }

        if (work_id < stream_size)
        {
            ++work_iter;

            // run the continuation
            stats.sample( RUN_EVENT );
            if (unit.run( stream ) == false)
            {
                // mark this unit as invalid
                work_id = invalid_unit;

                // sample the number of iterations this unit has been running
                stats.sample_iterations( work_iter );
            }
        }
    }
}

///@} // WorkQueueDetail

} // namespace wq

// consume a stream of work units
//
template <
    typename WorkUnitT,
    uint32   BLOCKDIM>
template <typename WorkStream, typename WorkMover>
void WorkQueue<PersistentThreadsQueueTag,WorkUnitT,BLOCKDIM>::consume(const WorkStream stream, const WorkMover, WorkQueueStats* stats)
{
    // compute the number of blocks we are going to launch
    const uint32 max_blocks = (uint32)cuda::max_active_blocks( wq::persistent_threads_work_queue_kernel<BLOCKDIM,WorkUnit,WorkStream>, BLOCKDIM, 0u );
    const uint32 n_blocks   = nvbio::max( nvbio::min( max_blocks, m_capacity / BLOCKDIM ), 1u );

    // resize and reset the work pool counter
    m_pool.resize(1);
    m_pool[0] = 0u;

    // compute the maximum number of tolerated inactive lanes, given the specified minimum utilization
    const uint32 min_active_lanes   = uint32( m_min_utilization * cuda::Arch::WARP_SIZE );
    const uint32 max_inactive_lanes = cuda::Arch::WARP_SIZE - min_active_lanes;

    // launch the consuming kernel
    wq::persistent_threads_work_queue_kernel<BLOCKDIM,WorkUnit,WorkStream> <<<n_blocks,BLOCKDIM>>>( thrust::raw_pointer_cast( &m_pool.front() ), max_inactive_lanes, stream, view( stats ) );
}

///@} // WorkQueue

} // namespace cuda
} // namespace nvbio
