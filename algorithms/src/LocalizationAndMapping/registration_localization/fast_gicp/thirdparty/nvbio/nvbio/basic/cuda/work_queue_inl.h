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

namespace wq {

///@addtogroup WorkQueue
///@{

///@addtogroup WorkQueueDetail
///@{

template <
    uint32   BLOCKDIM,
    typename WorkUnitT,
    typename WorkStreamT>
__global__
void inplace_work_queue_kernel(const WorkStreamT stream, WorkQueueStats::View stats)
{
    typedef WorkUnitT WorkUnit;

    const uint32 grid_threads = gridDim.x * BLOCKDIM;
    const uint32 thread_id    = threadIdx.x + blockIdx.x*BLOCKDIM;

    // place a work-unit in local memory
    WorkUnit unit;

    const uint32 stream_end = stream.size();

    // let this CTA fetch all tiles at a grid-threads stride, starting from blockIdx.x*BLOCKDIM
    for (uint32 stream_begin = 0; stream_begin < stream_end; stream_begin += grid_threads)
    {
        const uint32 work_id = thread_id + stream_begin;

        if (work_id < stream_end)
        {
            // fetch the work unit
            stream.get( work_id, &unit, make_uint2( thread_id, 0u ) );
            stats.sample( STREAM_EVENT );

            // keep an iteration counter
            uint32 work_iter = 0;

            // run the unit until completion
            do { stats.sample( RUN_EVENT ); ++work_iter; } while (unit.run( stream ));

            // sample the number of iterations this unit has been running
            stats.sample_iterations( work_iter );
        }
    }
}

///@} // WorkQueueDetail

} // namespace wq

// consume a stream of work units
//
template <
    typename PolicyTag,
    typename WorkUnitT,
    uint32   BLOCKDIM>
template <typename WorkStream, typename WorkMover>
void WorkQueue<PolicyTag,WorkUnitT,BLOCKDIM>::consume(const WorkStream stream, const WorkMover, WorkQueueStats* stats)
{
    // compute the number of blocks we are going to launch
    const uint32 n_blocks = (uint32)cuda::max_active_blocks( wq::inplace_work_queue_kernel<BLOCKDIM,WorkUnit,WorkStream>, BLOCKDIM, 0u );

    // launch the consuming kernel
    wq::inplace_work_queue_kernel<BLOCKDIM,WorkUnit,WorkStream> <<<n_blocks,BLOCKDIM>>>( stream, view( stats ) );
}


// sample utilization
//
NVBIO_FORCEINLINE NVBIO_DEVICE
void WorkQueueStats::View::sample(const WorkQueueStatsEvent type)
{
    if (valid() == false)
        return;

    const uint32 active_mask  = __ballot(true);
    const uint32 active_count = __popc(active_mask);
    if (__popc(active_mask >> warp_tid()) == 1u)
    {
        atomicAdd( active_lanes + type, active_count );
        atomicAdd( issued_warps + type, 1u );
    };
}

// sample iterations
//
NVBIO_FORCEINLINE NVBIO_DEVICE
void WorkQueueStats::View::sample_iterations(const uint32 i)
{
    if (valid() == false)
        return;

    atomicAdd( iterations,    i );              // add to the total sum
    atomicMax( (uint32*)(iterations+1u), i );   // take the maximum
    atomicAdd( iterations+2u, 1u );             // increase the event counter
}

///@} // WorkQueue

} // namespace cuda
} // namespace nvbio
