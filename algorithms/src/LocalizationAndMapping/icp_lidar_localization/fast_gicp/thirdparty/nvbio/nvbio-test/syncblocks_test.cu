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

// syncblocks_test.cu
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
#include <nvbio/basic/cuda/syncblocks.h>

namespace nvbio {

__global__
void print_kernel(const uint32 n_barriers, cuda::syncblocks barrier, uint32* queue_ptr, uint2* queue)
{
    for (uint32 i = 0; i < n_barriers; ++i)
    {
        if (threadIdx.x == 0)
        {
            const uint32 slot = atomicAdd( queue_ptr, 1u );
            queue[slot] = make_uint2( i, blockIdx.x );
            //NVBIO_CUDA_DEBUG_ASSERT( slot >= i*gridDim.x, "block[%u] got slot %u at iteration %u\n", blockIdx.x, slot, i );
        }

        barrier.enact();
    }
}
__global__
void speed_kernel(const uint32 n_barriers, cuda::syncblocks barrier, uint2* output)
{
    for (uint32 i = 0; i < n_barriers; ++i)
        barrier.enact();

    output[blockIdx.x] = make_uint2( blockIdx.x, 0 );
}

int syncblocks_test()
{
    const uint32 n_barriers = 100;
    cuda::syncblocks_storage barrier_st;

    cuda::syncblocks barrier = barrier_st.get();

    log_info( stderr, "syncblocks test... started\n" );

    const uint32 blockdim = 128;
    const uint32 n_blocks = max_active_blocks( print_kernel, blockdim, 0u );
    log_info( stderr, "  %u blocks\n", n_blocks );

    thrust::device_vector<uint32> dqueue_head( 1u );
    thrust::device_vector<uint2>  dqueue( n_barriers*n_blocks );

    uint32* dqueue_head_ptr = thrust::raw_pointer_cast( &dqueue_head.front() );
    uint2*  dqueue_ptr      = thrust::raw_pointer_cast( &dqueue.front() );

    thrust::host_vector<uint2> hqueue;
    log_info( stderr, "  correctness test... started\n" );

    for (uint32 i = 0; i < 20; ++i)
    {
        // initialize the queue pointer
        dqueue_head[0] = 0;

        // call the testing kernel
        print_kernel<<<n_blocks,blockdim>>>( n_barriers, barrier, dqueue_head_ptr, dqueue_ptr );
        cudaDeviceSynchronize();

        nvbio::cuda::thrust_copy_vector(hqueue, dqueue);

        for (uint32 n = 0; n < n_barriers; ++n)
        {
            for (uint32 j = 0; j < n_blocks; ++j)
            {
                const uint2 val = hqueue[n*n_blocks + j];
                if (val.x != n)
                {
                    log_error( stderr, "  found (%u,%u) at position %u:%u, launch %u\n", val.x, val.y, n, j, i );
                    return 1;
                }
            }
        }
    }
    log_info( stderr, "  correctness test... done\n" );

    const uint32 n_tests = 100;

    log_info( stderr, "  speed test... started\n" );

    Timer timer;
    timer.start();

    for (uint32 i = 0; i < n_tests; ++i)
        speed_kernel<<<n_blocks,blockdim>>>( n_barriers, barrier, dqueue_ptr+1 );

    cudaDeviceSynchronize();
    timer.stop();

    const float time = timer.seconds() / (n_tests*n_barriers);

    log_info( stderr, "  speed test... done: %.1f ns\n", time * 1.0e6f );

    log_info( stderr, "syncblocks test... done\n" );
    return 0;
}

} // namespace nvbio
