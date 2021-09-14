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

// condition_test.cu
//
#define NVBIO_CUDA_DEBUG

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <nvbio/basic/timer.h>
#include <nvbio/basic/console.h>
#include <nvbio/basic/vector.h>
#include <nvbio/basic/cuda/arch.h>
#include <nvbio/basic/cuda/condition.h>
#include <cub/cub.cuh>

namespace nvbio {
namespace condition {

__global__
void scan_kernel(const uint32 n_tile_grids, cuda::condition_set_view conditions, uint32* block_values, const uint32 initial_state)
{
    const uint32 prev_block = blockIdx.x ? blockIdx.x-1    : gridDim.x-1;
          uint32 prev_state = blockIdx.x ? initial_state+1 : initial_state;

    for (uint32 i = 0; i < n_tile_grids; ++i, ++prev_state)
    {
        // compute the tile number
        const uint32 tile_idx = blockIdx.x + i * gridDim.x;

        if (threadIdx.x == 0)
        {
            if (tile_idx)
            {
                // wait on the previous block to post its value
                conditions[ prev_block ].wait( prev_state  );

                block_values[tile_idx] = block_values[tile_idx-1] + 1;
            }
            else
            {
                // set the value for the first tile
                block_values[0] = 0;
            }

            // release the condition for this block
            conditions[ blockIdx.x ].signal(); // equivalent to set( initial_state+i+1 )
        }
    }
}

__global__
void chained_kernel(const uint32 n_tile_grids, cuda::condition_set_view conditions, const uint32 initial_state)
{
    const uint32 prev_block = blockIdx.x ? blockIdx.x-1    : gridDim.x-1;
          uint32 prev_state = blockIdx.x ? initial_state+1 : initial_state;

    for (uint32 i = 0; i < n_tile_grids; ++i, ++prev_state)
    {
        // compute the tile number
        const uint32 tile_idx = blockIdx.x + i * gridDim.x;

        if (threadIdx.x == 0)
        {
            if (tile_idx)
            {
                // wait on the previous block to be ready
                conditions[ prev_block ].wait( prev_state  );
            }

            // release the condition for this block
            conditions[ blockIdx.x ].signal(); // equivalent to set( initial_state+i+1 )
        }
    }
}

template <uint32 BLOCKDIM, bool DO_WORK>
__global__
void fast_scan_kernel(const uint32 n_tile_grids, cuda::condition_set_view conditions, volatile uint32* partials, volatile uint32* prefixes, const uint32 initial_state)
{
    //
    // This scan skeleton performs longer range chaining rather than chaining each CTA
    // to its direct predecessor: the idea is that each CTA writes out its partial,
    // and then checks for the availability of prefix of predecessor (without blocking),
    // if not available, it waits for the previous BLOCKDIM-1 partials and the prefix BLOCKDIM
    // CTAs away, and uses all its threads to reduce them.
    // Hence, CTA[i] depends on the prefix of a CTA[i-BLOCKDIM].
    //
    typedef cub::BlockReduce<uint32,BLOCKDIM> BlockReduce;

    const uint32 PARTIAL_READY = initial_state + 1;
    const uint32 PREFIX_READY  = initial_state + 2;
    
    __shared__ typename BlockReduce::TempStorage smem_storage;
    __shared__ uint32 previous_done;

    const bool is_thread0 = (threadIdx.x == 0);

    for (uint32 i = 0; i < n_tile_grids; ++i)
    {
        //__syncthreads();

        // compute the tile number
        const uint32 tile_idx = blockIdx.x + i * gridDim.x;

        // write out the partial for this tile
        if (DO_WORK)
        {
            if (is_thread0)
                partials[tile_idx] = 1;
        }

        if (is_thread0 && tile_idx)
            previous_done = conditions[tile_idx-1].test( PREFIX_READY );

        __syncthreads();

        if (tile_idx == 0)
        {
            // set the value for the first tile
            if (DO_WORK)
            {
                if (is_thread0)
                    prefixes[0] = 1;
            }
        }
        else if (previous_done)
        {
            // sum to previous prefix
            if (DO_WORK)
            {
                if (is_thread0)
                    prefixes[tile_idx] = prefixes[tile_idx-1] + 1;
            }
        }
        else
        {
            // release the condition variable for the partial
            if (is_thread0)
                conditions[tile_idx].set( PARTIAL_READY );

            int32 prefix  = 0;

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
                    if (conditions[ prefix_tile + threadIdx.x ].test( PREFIX_READY ))
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
                        if (DO_WORK)
                            partial = prefixes[ prefix_tile ];
                    }
                    else
                    {
                        // wait on the partials
                        conditions[ prefix_tile + threadIdx.x ].wait( PARTIAL_READY );

                        if (DO_WORK)
                            partial = partials[ prefix_tile + threadIdx.x ];
                    }
                }

                if (DO_WORK)
                {
                    // reduce the prefixes
                    prefix += BlockReduce( smem_storage ).Sum( partial );
                }

                last_tile = prefix_tile;
            }
            while (prefix_tile && !previous_done);

            if (DO_WORK)
            {
                if (is_thread0)
                {
                    // write out the final values
                    prefixes[tile_idx] = prefix + 1;
                }
            }
        }

        // release the condition for the scanned value for this tile
        if (is_thread0)
            conditions[tile_idx].set( PREFIX_READY );
    }
}

} // condition namespace

int condition_test()
{
    const uint32 n_tile_grids = 100;

    log_info( stderr, "condition test... started\n" );

    const uint32 blockdim = 128;
    const uint32 n_blocks = (uint32)cuda::max_active_blocks( condition::scan_kernel, blockdim, 0u );

    cuda::condition_set_storage condition_st( n_blocks );
    cuda::condition_set_view    condition_set = condition_st.get();

    log_info( stderr, "  %u blocks\n", n_blocks );

    thrust::device_vector<uint32> dvalues( n_tile_grids*n_blocks );

    uint32* dvalues_ptr = thrust::raw_pointer_cast( &dvalues.front() );

    thrust::host_vector<uint32> hvalues;
    log_info( stderr, "  correctness test... started\n" );

    for (uint32 i = 0; i < 20; ++i)
    {
        // call the testing kernel
        condition::scan_kernel<<<n_blocks,blockdim>>>( n_tile_grids, condition_set, dvalues_ptr, i*n_tile_grids );
        cudaDeviceSynchronize();

        nvbio::cuda::thrust_copy_vector(hvalues, dvalues);

        for (uint32 n = 0; n < n_tile_grids*n_blocks; ++n)
        {
            const uint32 val = hvalues[n];
            if (val != n)
            {
                log_error( stderr, "  found %u at position %u, launch %u\n", val, n, i );
                return 1;
            }
        }
    }
    log_info( stderr, "  correctness test... done\n" );

    const uint32 n_tests = 20;

    log_info( stderr, "  speed test... started\n" );

    condition_st.set(0);
    cudaDeviceSynchronize();

    Timer timer;
    timer.start();

    for (uint32 i = 0; i < n_tests; ++i)
        condition::chained_kernel<<<n_blocks,blockdim>>>( n_tile_grids, condition_set, i*n_tile_grids );

    cudaDeviceSynchronize();
    timer.stop();


    const float time = timer.seconds() / float(n_tests*n_tile_grids);

    log_info( stderr, "  speed test... done:\n    %.3f ns\n    %.3f ns/CTA\n    %.1fM CTAs retired/s\n",
        time * 1.0e6f,
        1.0e6f * (time/float(n_blocks)),
        1.0e-6f * (float(n_blocks)/time) );

    {
        const uint32 blockdim = 128;
        const uint32 n_blocks = (uint32)cuda::max_active_blocks( condition::fast_scan_kernel<blockdim,true>, blockdim, 0u );

        cuda::condition_set_storage condition_st( n_blocks*n_tile_grids );
        cuda::condition_set_view    condition_set = condition_st.get();
        cudaDeviceSynchronize();

        log_info( stderr, "  fast scan... started (%u CTAs)\n", n_blocks );

        thrust::device_vector<uint32> dpartials( n_tile_grids*n_blocks );
        uint32* dpartials_ptr = thrust::raw_pointer_cast( &dpartials.front() );

        for (uint32 i = 0; i < 20; ++i)
        {
            thrust::fill( dpartials.begin(), dpartials.end(), 0 );
            thrust::fill( dvalues.begin(), dvalues.end(), 0 );
            condition::fast_scan_kernel<blockdim,true><<<n_blocks,blockdim>>>( n_tile_grids, condition_set, dpartials_ptr, dvalues_ptr, i*2 );
            cudaDeviceSynchronize();

            nvbio::cuda::thrust_copy_vector(hvalues, dvalues);

            for (uint32 n = 0; n < n_tile_grids*n_blocks; ++n)
            {
                const uint32 val = hvalues[n];
                if (val != n+1)
                {
                    log_error( stderr, "  found %u at position %u, launch %u\n", val, n, i );
                    return 1;
                }
            }
        }

        condition_st.set(0);
        cudaDeviceSynchronize();

        Timer timer;
        timer.start();

        for (uint32 i = 0; i < n_tests; ++i)
            condition::fast_scan_kernel<blockdim,true><<<n_blocks,blockdim>>>( n_tile_grids, condition_set, dpartials_ptr, dvalues_ptr, i*2 );

        cudaDeviceSynchronize();
        timer.stop();
        {
            const float time = timer.seconds() / float(n_tests*n_tile_grids);

            log_info( stderr, "  fast scan test... done:\n    %.3f ns\n    %.3f ns/CTA\n    %.1fM CTAs retired/s\n",
                time * 1.0e6f,
                1.0e6f * (time/float(n_blocks)),
                1.0e-6f * (float(n_blocks)/time) );
        }

        log_info( stderr, "  fast chaining... started\n" );

        condition_st.set(0);
        cudaDeviceSynchronize();

        timer.start();

        for (uint32 i = 0; i < n_tests; ++i)
            condition::fast_scan_kernel<blockdim,false><<<n_blocks,blockdim>>>( n_tile_grids, condition_set, NULL, NULL, i*2 );

        cudaDeviceSynchronize();
        timer.stop();
        {
            const float time = timer.seconds() / float(n_tests*n_tile_grids);

            log_info( stderr, "  fast chaining test... done:\n    %.3f ns\n    %.3f ns/CTA\n    %.1fM CTAs retired/s\n",
                time * 1.0e6f,
                1.0e6f * (time/float(n_blocks)),
                1.0e-6f * (float(n_blocks)/time) );
        }
    }

    log_info( stderr, "condition test... done\n" );
    return 0;
}

} // namespace nvbio
