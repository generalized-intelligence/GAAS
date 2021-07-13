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

namespace wq {

///@addtogroup WorkQueueDetail
///@{

template <
    uint32   BLOCKDIM,
    typename WorkUnitT,
    typename WorkStreamT,
    typename WorkMoverT>
__global__
void work_queue_kernel(
    const uint32                                                    n_tile_grids,
    typename WorkQueue<OrderedQueueTag,WorkUnitT,BLOCKDIM>::Context context,
    const WorkStreamT                                               stream,
    const WorkMoverT                                                mover)
{
    //
    // This kernel consumes a stream of input tasks, which can either stop or produce one
    // more task, i.e. a continuation.
    // At each iteration, in order to gather SIMT utilization while mantaining as much data
    // coherence as possible, the continuations are compacted in a queue keeping input ordering
    // for the next round of processing.
    // This means that we need to essentially write an intra-kernel scan, or prefix sum, on
    // the number of continuations produced at each round, in order to locate the output slot
    // of each continuation.
    //
    typedef WorkUnitT WorkUnit;

    typedef cub::BlockReduce<uint32,BLOCKDIM> BlockReduce;
    typedef cub::BlockScan<uint32,BLOCKDIM>   BlockBitScan;

    const uint32 grid_threads   = gridDim.x * BLOCKDIM;
    const uint32 queue_capacity = grid_threads * n_tile_grids;
    const uint32 thread_id      = threadIdx.x + blockIdx.x*BLOCKDIM;

    uint32              work_queue_id0   = 0u;
    WorkUnit*           work_queue0      = context.m_work_queue;
    WorkUnit*           work_queue1      = context.m_work_queue + queue_capacity;
    volatile uint32*    work_queue_size0 = context.m_work_queue_size;
    volatile uint32*    work_queue_size1 = context.m_work_queue_size + 1;

    __shared__ typename BlockBitScan::TempStorage scan_smem_storage;
    __shared__ typename BlockReduce::TempStorage  reduce_smem_storage;

    const uint32 COND_THRESHOLD = 10000000;
    enum TileState {
        PARTIAL_READY = 1,
        PREFIX_READY  = 2,
    };

    condition_set_view conditions = context.m_conditions;

    #define SINGLE_CTA_SCAN
    #if !defined(SINGLE_CTA_SCAN)
    __shared__ volatile uint32 previous_done;
    #else
    uint8* continuations = context.m_continuations;
    #endif

    //#define QUEUE_GATHER
    #if defined(QUEUE_GATHER)
    uint32* source_ids = context.m_source_ids;
    #endif

    volatile uint32* partials   = context.m_partials;
    volatile uint32* prefixes   = context.m_prefixes;

    const bool is_thread0 = (threadIdx.x == 0);

          uint32 stream_begin = 0;
    const uint32 stream_end   = stream.size();

    uint32 iteration = 0;

    while (1)
    {
        // make sure all threads see the same queue pointers
        context.m_syncblocks.enact();

        uint32 in_work_queue_size = *work_queue_size0;

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
                    stream.get( stream_begin + work_id - in_work_queue_size, work_queue0 + work_id, make_uint2( work_id, work_queue_id0 ) );

                in_work_queue_size += n_loaded;
                stream_begin       += n_loaded;
            }
        }

        // bail-out if there's no more work
        if (in_work_queue_size == 0)
            return;

        // check whether all threads are active
        NVBIO_CUDA_DEBUG_ASSERT( (__syncthreads_and(true) == true), "error: not all threads are active at block: %u\n", blockIdx.x );

        // compute how many tiles are needed to cover the input queue
        const uint32 n_active_tile_grids = (in_work_queue_size + grid_threads-1) / grid_threads;
        const uint32 n_active_tiles      = n_active_tile_grids * gridDim.x;

    #if defined(SINGLE_CTA_SCAN)
        //
        // This is essentially a scan skeleton that uses a single CTA (blockIdx.x = 0) to scan the partials
        // from other CTAs while they arrive.
        //

        // loop across the active tiles
        for (uint32 i = 0; i < n_active_tile_grids; ++i)
        {
            // make sure all threads get here at the same time
            __syncthreads();

            // compute the tile number
            const uint32 tile_idx = blockIdx.x + gridDim.x * i;
            const uint32 work_id  = thread_id + grid_threads * i;

            bool has_continuation = false;

            // execute this thread's work unit
            if (work_id < in_work_queue_size)
                has_continuation = work_queue0[ work_id ].run( stream );

            continuations[ work_id ] = has_continuation ? 1u : 0u;

            // scan the number of continuations in this block and compute their aggregate count
            const uint32 block_aggregate = BlockReduce( reduce_smem_storage ).Sum( has_continuation ? 1 : 0 );

            // write out the partial aggregate for this tile
            if (is_thread0)
            {
                partials[tile_idx] = block_aggregate;

                // set the partial ready flag
                conditions[tile_idx].set( iteration*2 + PARTIAL_READY );
            }
        }

        // scan the partials
        if (blockIdx.x == 0)
        {
            // make sure all threads get here at the same time
            __syncthreads();

            uint32 carry = 0;

            // we have to scan a grid's worth of block partials
            for (uint32 tile_begin = 0; tile_begin < n_active_tiles; tile_begin += BLOCKDIM)
            {
                const uint32 tile_x     = tile_begin + threadIdx.x;
                const bool   tile_valid = (tile_x < n_active_tiles);

                // check whether this tile is ready
                if (tile_valid)
                {
                    if (!conditions[tile_x].wait( iteration*2 + PARTIAL_READY, COND_THRESHOLD ))
                        printf("PARTIAL_READY test failed for tile %u/%u at iteration %u\n", tile_x, n_active_tiles, iteration);
                }

                const uint32 partial = tile_valid ? partials[tile_x] : 0u;

                uint32 partial_scan;
                uint32 partial_aggregate;

                // scan the partials
                BlockBitScan( scan_smem_storage ).ExclusiveSum( partial, partial_scan, partial_aggregate );

                // write the prefixes out
                if (tile_valid)
                {
                    prefixes[tile_x] = carry + partial_scan;
                    conditions[tile_x].set( iteration*2 + PREFIX_READY );
                }

                carry += partial_aggregate;
            }

            if (is_thread0)
                *work_queue_size1 = carry;

            // make sure all threads get here at the same time
            __syncthreads();
        }

        for (uint32 i = 0; i < n_active_tile_grids; ++i)
        {
            // make sure all threads get here at the same time
            __syncthreads();

            // compute the tile number
            const uint32 tile_idx = blockIdx.x + gridDim.x * i;
            const uint32 work_id  = thread_id + grid_threads * i;

            // check whether this tile is ready
            if (!conditions[tile_idx].wait( iteration*2 + PREFIX_READY, COND_THRESHOLD ))
                printf("PREFIX_READY test failed for tile %u/%u at iteration %u\n", tile_idx, n_active_tiles, iteration);

            const uint32 prefix = prefixes[tile_idx];

            const uint32 has_continuation = continuations[ work_id ];

            uint32 block_scan;
            uint32 block_aggregate;

            // scan the number of continuations in this block and compute their aggregate count
            BlockBitScan( scan_smem_storage ).ExclusiveSum( has_continuation, block_scan, block_aggregate );

            // at this point this CTA knows where to output its continuations in the output queue
            if (has_continuation)
            {
                #if defined(QUEUE_GATHER)
                source_ids[ prefix + block_scan ] = work_id;
                #else
                // copy this work unit to its destination
                mover.move(
                    stream,
                    make_uint2( work_id, work_queue_id0 ),                          &work_queue0[ work_id ],
                    make_uint2( prefix + block_scan, work_queue_id0 ? 0u : 1u ),    &work_queue1[ prefix + block_scan ] );
                #endif
            }
        }
    #else // !SINGLE_CTA_SCAN
        //
        // This is essentially a scan skeleton that performs CTA chaining using an adaptive lookback
        // strategy: the idea is that each CTA writes out its partial, and then checks for the availability
        // of the prefix of its predecessor (without blocking); if not available, it starts using all its
        // threads to look for a ready predecessor prefix, and scan the intermediate partials.
        //

        for (uint32 i = 0; i < n_tile_grids; ++i)
        {
            __syncthreads();

            // compute the tile number
            const uint32 tile_idx = blockIdx.x + gridDim.x * i;

            const uint32 work_id = thread_id + grid_threads * i;

            bool has_continuation = false;

            // execute this thread's work unit
            if (work_id < in_work_queue_size)
                has_continuation = work_queue0[ work_id ].run( stream );

            uint32 block_scan;
            uint32 block_aggregate;

            // scan the number of continuations in this block and compute their aggregate count
            BlockBitScan( scan_smem_storage ).ExclusiveSum( has_continuation ? 1 : 0, block_scan, block_aggregate );

            // write out the partial aggregate for this tile
            if (is_thread0)
                partials[tile_idx] = block_aggregate;

            // quickly check if the prefix for the previous tile is ready
            if (is_thread0 && tile_idx)
                previous_done = conditions[tile_idx-1].test( iteration*2 + PREFIX_READY );

            // make sure all threads see 'previous_done'
            __syncthreads();

            int32 prefix  = 0;

            if (tile_idx == 0)
            {
                // set the value for the first tile
                if (is_thread0)
                    prefixes[0] = block_aggregate;
            }
        #if 0   // simple chaining
            else if (tile_idx)
            {
                if (is_thread0)
                {
                    if (!conditions[tile_idx-1].wait( iteration*2 + PREFIX_READY, COND_THRESHOLD ))
                        printf("PREFIX_READY test failed for tile %u/%u at iteration %u\n", tile_idx-1, n_active_tiles, iteration);

                    prefix = prefixes[tile_idx-1];

                    // sum to previous prefix
                    if (is_thread0)
                        prefixes[tile_idx] = prefix + block_aggregate;
                }
            }
        #else   // adaptive lookback
            else if (previous_done)
            {
                prefix = prefixes[tile_idx-1];

                // sum to previous prefix
                if (is_thread0)
                    prefixes[tile_idx] = prefix + block_aggregate;
            }
            else
            {
                // release the condition variable for the partial
                if (is_thread0)
                    conditions[tile_idx].set( iteration*2 + PARTIAL_READY );

                int32 last_tile   = tile_idx;
                int32 prefix_tile = tile_idx;

                // keep looking back until we find a 'ready' prefix
                do
                {
                    //
                    // lookback up to BLOCKDIM predecessors in parallel, check if any
                    // of them is done (i.e. their prefix is ready), and otherwise
                    // wait on their partial to arrive.
                    //

                    previous_done = 0;
                    __syncthreads();

                    // compute the first tile in this batch
                    prefix_tile = nvbio::max( int32( prefix_tile - blockDim.x ), 0 );

                    // check if the any of the predecessors in this block is done
                    if (prefix_tile + threadIdx.x < last_tile)
                    {
                        if (conditions[ prefix_tile + threadIdx.x ].test( iteration*2 + PREFIX_READY ))
                            previous_done = prefix_tile + threadIdx.x;
                    }

                    __syncthreads();

                    // let all threads update the prefix tile
                    if (previous_done)
                        prefix_tile = previous_done;

                    int32 partial = 0;

                    // lookback the predecessors in parallel
                    if (prefix_tile + threadIdx.x < last_tile)
                    {
                        if (previous_done && threadIdx.x == 0)
                        {
                            // let thread0 read the ready prefix
                            partial = prefixes[ prefix_tile ];
                        }
                        else
                        {
                            // wait on the partials
                            if (!conditions[ prefix_tile + threadIdx.x ].wait( iteration*2 + PARTIAL_READY, COND_THRESHOLD ))
                                printf("PARTIAL_READY test failed for tile %u at tile %u/%u at iteration %u\n", prefix_tile + threadIdx.x, tile_idx-1, n_active_tiles, iteration);

                            partial = partials[ prefix_tile + threadIdx.x ];
                        }
                    }

                    // reduce the prefixes
                    prefix += BlockReduce( reduce_smem_storage ).Sum( partial );

                    last_tile = prefix_tile;
                }
                while (prefix_tile && !previous_done);

                if (is_thread0)
                {
                    // write out the final values
                    prefixes[tile_idx] = prefix + block_aggregate;
                }
            }
        #endif
            // share the prefix with the rest of the CTA
            __syncthreads();
            if (is_thread0)
                previous_done = prefix;
            __syncthreads();
            prefix = previous_done;

            // write the output queue size
            if (tile_idx == n_active_tiles-1 && is_thread0)
                *work_queue_size1 = prefix + block_aggregate;

            // at this point this CTA knows where to output its continuations in the output queue
            if (has_continuation)
            {
                #if defined(QUEUE_GATHER)
                source_ids[ prefix + block_scan ] = work_id;
                #else
                // copy this work unit to its destination
                mover.move(
                    stream,
                    make_uint2( work_id, work_queue_id0 ),                          &work_queue0[ work_id ],
                    make_uint2( prefix + block_scan, work_queue_id0 ? 0u : 1u ),    &work_queue1[ prefix + block_scan ] );
                #endif
            }

            // release the condition for the scanned value for this tile
            if (is_thread0)
                conditions[tile_idx].set( iteration*2 + PREFIX_READY );
        }

        __syncthreads();

        // block all CTAs until the last tile has been processed
        if (!conditions[n_active_tiles-1].wait( iteration*2 + PREFIX_READY, COND_THRESHOLD ))
            printf("PREFIX_READY test failed for last tile (%u) at iteration %u\n", n_active_tiles-1, iteration);
    #endif // !SINGLE_CTA_SCAN

        // we can now swap the queue pointers
        {
            WorkUnit* tmp = work_queue0;
            work_queue0   = work_queue1;
            work_queue1   = tmp;
        }
        {
            volatile uint32* tmp = work_queue_size0;
            work_queue_size0 = work_queue_size1;
            work_queue_size1 = tmp;
        }
        work_queue_id0 = work_queue_id0 ? 0u : 1u;

        ++iteration;

        // gather the compacted data
        #if defined(QUEUE_GATHER)
        {
            // make sure all threads see the same queue pointers
            context.m_syncblocks.enact();

            // compute how many tiles are needed to cover the input queue
            const uint32 out_grid_size       = *work_queue_size0;
            const uint32 n_active_tile_grids = (out_grid_size + grid_threads-1) / grid_threads;

            for (uint32 i = 0; i < n_active_tile_grids; ++i)
            {
                // compute the tile number
                const uint32 work_id = thread_id + grid_threads * i;

                if (work_id < out_grid_size)
                {
                    // fetch the source id
                    const uint32 src_id = source_ids[ work_id ];

                    mover.move(
                        stream,
                        make_uint2( src_id,  work_queue_id0 ? 0u : 1u ),    &work_queue1[ src_id ],
                        make_uint2( work_id, work_queue_id0 ? 1u : 0u ),    &work_queue0[ work_id ] );
                }
            }
        }
        #endif // QUEUE_GATHER
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
void WorkQueue<OrderedQueueTag,WorkUnitT,BLOCKDIM>::consume(const WorkStream stream, const WorkMover mover)
{
    //const uint32 stream_size = stream.size();

    // compute the number of blocks we are going to launch
    const uint32 n_blocks = (uint32)cuda::max_active_blocks( wq::work_queue_kernel<BLOCKDIM,WorkUnit,WorkStream,WorkMover>, BLOCKDIM, 0u );
    const uint32 grid_threads = n_blocks * BLOCKDIM;

    const uint32 n_tile_grids = m_capacity / grid_threads;
    m_condition_set.resize( n_blocks*n_tile_grids );
    m_partials.resize( n_blocks*n_tile_grids );
    m_prefixes.resize( n_blocks*n_tile_grids );
    m_continuations.resize( grid_threads*n_tile_grids );
    m_source_ids.resize( grid_threads*n_tile_grids );
    m_work_queue.resize( grid_threads*n_tile_grids * 2 );   // alloc space for an input and an output queue
    m_work_queue_size.resize( 2 );                          // alloc space for an input and an output queue
    m_syncblocks.clear();

    m_work_queue_size[0] = 0;

    // launch the consuming kernel
    wq::work_queue_kernel<BLOCKDIM,WorkUnit,WorkStream,WorkMover> <<<n_blocks,BLOCKDIM>>>( n_tile_grids, get_context(), stream, mover );
}

///@} // WorkQueue

} // namespace cuda
} // namespace nvbio
