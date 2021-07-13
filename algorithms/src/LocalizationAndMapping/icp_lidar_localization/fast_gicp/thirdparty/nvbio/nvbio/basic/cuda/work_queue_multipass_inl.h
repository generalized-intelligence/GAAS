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
#include <thrust/scan.h>

namespace nvbio {
namespace cuda {

///@addtogroup WorkQueue
///@{

namespace wq {

///@addtogroup WorkQueueDetail
///@{

template <
    uint32   BLOCKDIM,
    typename WorkUnitT,
    typename WorkStreamT,
    bool     DO_LOADS>
__global__
void mk_work_queue_kernel(
    const uint32                                                        n_tile_grids,
    typename WorkQueue<MultiPassQueueTag,WorkUnitT,BLOCKDIM>::Context   context,
    const uint32                                                        in_queue_id,
    const uint32                                                        in_queue_size,
    const WorkStreamT                                                   stream,
          uint32                                                        stream_begin)
{
    typedef WorkUnitT WorkUnit;

    const uint32 grid_threads   = gridDim.x * BLOCKDIM;
    const uint32 queue_capacity = grid_threads * n_tile_grids;
    const uint32 thread_id      = threadIdx.x + blockIdx.x*BLOCKDIM;

    WorkUnit*  work_queue    = context.m_work_queue + queue_capacity*in_queue_id;
    uint32*    continuations = context.m_continuations;

    const uint32 stream_end = stream.size();

    uint32 in_work_queue_size = in_queue_size;

    if (DO_LOADS)
    {
        // try to load more work to do
        for (uint32 i = 0; i < n_tile_grids; ++i)
        {
            const uint32 work_begin = grid_threads * i;
            const uint32 work_id    = thread_id + work_begin;

            // if the work queue is not full, and there's remaining work in the stream, fetch some more
            if ((work_begin                <= in_work_queue_size) &&
                (work_begin + grid_threads >  in_work_queue_size) &&
                (stream_begin < stream_end))
            {
                const uint32 n_loaded = nvbio::min( stream_end - stream_begin, grid_threads - (in_work_queue_size - work_begin) );

                // fetch a new work unit
                if ((work_id >= in_work_queue_size) &&
                    (work_id -  in_work_queue_size < n_loaded))
                    stream.get( stream_begin + work_id - in_work_queue_size, work_queue + work_id, make_uint2( work_id, in_queue_id ) );

                in_work_queue_size += n_loaded;
                stream_begin       += n_loaded;
            }
        }
    }

    // compute how many tiles are needed to cover the input queue
    const uint32 n_active_tile_grids = (in_work_queue_size + grid_threads-1) / grid_threads;

    // execute the queued work units
    for (uint32 i = 0; i < n_active_tile_grids; ++i)
    {
        const uint32 work_id = thread_id + grid_threads * i;

        // execute this thread's work unit
        if (work_id < in_work_queue_size)
        {
            const bool has_continuation = work_queue[ work_id ].run( stream );
            continuations[ work_id ] = has_continuation ? 1u : 0u;
        }
    }
}

template <
    uint32   BLOCKDIM,
    typename WorkUnitT,
    typename WorkStreamT>
__global__
void mk_load_kernel(
    const uint32                                                        n_tile_grids,
    typename WorkQueue<MultiPassQueueTag,WorkUnitT,BLOCKDIM>::Context   context,
    const uint32                                                        in_queue_id,
    const uint32                                                        in_queue_size,
    const WorkStreamT                                                   stream,
          uint32                                                        stream_begin)
{
    typedef WorkUnitT WorkUnit;

    const uint32 grid_threads   = gridDim.x * BLOCKDIM;
    const uint32 queue_capacity = grid_threads * n_tile_grids;
    const uint32 thread_id      = threadIdx.x + blockIdx.x*BLOCKDIM;

    WorkUnit* work_queue = context.m_work_queue + queue_capacity*in_queue_id;

    const uint32 stream_end = stream.size();

    uint32 in_work_queue_size = in_queue_size;

    // try to load more work to do
    for (uint32 i = 0; i < n_tile_grids; ++i)
    {
        const uint32 work_begin = grid_threads * i;
        const uint32 work_id    = thread_id + work_begin;

        // if the work queue is not full, and there's remaining work in the stream, fetch some more
        if ((work_begin                <= in_work_queue_size) &&
            (work_begin + grid_threads >  in_work_queue_size) &&
            (stream_begin < stream_end))
        {
            const uint32 n_loaded = nvbio::min( stream_end - stream_begin, grid_threads - (in_work_queue_size - work_begin) );

            // fetch a new work unit
            if ((work_id >= in_work_queue_size) &&
                (work_id -  in_work_queue_size < n_loaded))
                stream.get( stream_begin + work_id - in_work_queue_size, work_queue + work_id, make_uint2( work_id, in_queue_id ) );

            in_work_queue_size += n_loaded;
            stream_begin       += n_loaded;
        }
    }
}

template <
    uint32   BLOCKDIM,
    typename WorkUnitT,
    typename WorkStream,
    typename WorkMover>
__global__
void mk_move_kernel(
    const uint32                                                        n_tile_grids,
    typename WorkQueue<MultiPassQueueTag,WorkUnitT,BLOCKDIM>::Context   context,
    const uint32                                                        in_queue_id,
    const uint32                                                        in_queue_size,
    const WorkStream                                                    stream,
    const WorkMover                                                     mover)
{
    typedef WorkUnitT WorkUnit;

    const uint32 grid_threads   = gridDim.x * BLOCKDIM;
    const uint32 queue_capacity = grid_threads * n_tile_grids;
    const uint32 thread_id      = threadIdx.x + blockIdx.x*BLOCKDIM;

    WorkUnit*     in_work_queue  = context.m_work_queue + queue_capacity*(in_queue_id ? 1 : 0);
    WorkUnit*     out_work_queue = context.m_work_queue + queue_capacity*(in_queue_id ? 0 : 1);
    const uint32* continuations  = context.m_continuations;

    // compute how many tiles are needed to cover the input queue
    const uint32 n_active_tile_grids = (in_queue_size + grid_threads-1) / grid_threads;

    // move the continuations from the input queue to the output queue
    for (uint32 i = 0; i < n_active_tile_grids; ++i)
    {
        const uint32 work_id = thread_id + grid_threads * i;

        if (work_id < in_queue_size)
        {
            const uint32 prev_slot = work_id ? continuations[ work_id-1 ] : 0u;
            const uint32 next_slot = continuations[ work_id ];
            const bool has_continuation = (next_slot > prev_slot);

            if (has_continuation)
            {
                // move the work-unit
                mover.move( stream,
                    make_uint2( work_id,    in_queue_id ? 1 : 0 ), &in_work_queue[ work_id ],
                    make_uint2( prev_slot,  in_queue_id ? 0 : 1 ), &out_work_queue[ prev_slot ] );
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
void WorkQueue<MultiPassQueueTag,WorkUnitT,BLOCKDIM>::consume(const WorkStream stream, const WorkMover mover, WorkQueueStats* stats)
{
    // compute the number of blocks we are going to launch
    const uint32 n_blocks = m_separate_loads ? 
        (uint32)cuda::max_active_blocks( wq::mk_work_queue_kernel<BLOCKDIM,WorkUnit,WorkStream,false>, BLOCKDIM, 0u ) :
        (uint32)cuda::max_active_blocks( wq::mk_work_queue_kernel<BLOCKDIM,WorkUnit,WorkStream,true>, BLOCKDIM, 0u );
    const uint32 grid_threads = n_blocks * BLOCKDIM;

    const uint32 n_tile_grids = m_capacity / grid_threads;
    const uint32 queue_capacity = grid_threads * n_tile_grids;

    m_continuations.resize( queue_capacity );
    m_work_queue.resize( queue_capacity * 2 ); // alloc space for an input and an output queue

    const uint32 stream_size = stream.size();

    uint32 in             = 0;
    uint32 in_queue_size  = 0;
    uint32 stream_begin   = 0;
    uint32 stream_end     = stream_size;

    typename thrust::device_vector<WorkUnit>::iterator in_queue_begin  = m_work_queue.begin();
    typename thrust::device_vector<WorkUnit>::iterator out_queue_begin = m_work_queue.begin() + queue_capacity;

    while (in_queue_size || stream_begin < stream_end)
    {
        const uint32 to_load = nvbio::min( queue_capacity - in_queue_size, stream_end - stream_begin );

        if (m_separate_loads)
        {
            // launch the loading kernel
            wq::mk_load_kernel<BLOCKDIM,WorkUnit,WorkStream> <<<n_blocks,BLOCKDIM>>>( n_tile_grids, get_context(), in, in_queue_size, stream, stream_begin );

            // update stream pointer
            stream_begin += to_load;

            // update input queue size
            in_queue_size += to_load;

            // launch the consuming kernel
            wq::mk_work_queue_kernel<BLOCKDIM,WorkUnit,WorkStream,false> <<<n_blocks,BLOCKDIM>>>( n_tile_grids, get_context(), in, in_queue_size, stream, stream_begin );
        }
        else
        {
            // launch the consuming kernel
            wq::mk_work_queue_kernel<BLOCKDIM,WorkUnit,WorkStream,true> <<<n_blocks,BLOCKDIM>>>( n_tile_grids, get_context(), in, in_queue_size, stream, stream_begin );

            // update stream pointer
            stream_begin += to_load;

            // update input queue size
            in_queue_size += to_load;
        }

        // compact the output
        thrust::inclusive_scan(
            m_continuations.begin(),
            m_continuations.begin() + in_queue_size,
            m_continuations.begin() );

        // synchronize on the scan
        cudaDeviceSynchronize();

        const uint32 out_queue_size = m_continuations[ in_queue_size - 1 ];

        // launch the movement kernel
        wq::mk_move_kernel<BLOCKDIM,WorkUnit,WorkStream,WorkMover> <<<n_blocks,BLOCKDIM>>>( n_tile_grids, get_context(), in, in_queue_size, stream, mover );

        // swap queue pointers
        in = in ? 0u : 1u;
        std::swap( in_queue_begin, out_queue_begin );

        in_queue_size = out_queue_size;
    }
}

///@} // WorkQueue

} // namespace cuda
} // namespace nvbio
