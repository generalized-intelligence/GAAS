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

// scan_test.cu
//

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <nvbio/basic/timer.h>
#include <nvbio/basic/console.h>
#include <nvbio/basic/cuda/arch.h>
#include <nvbio/basic/cuda/scan.h>
#include <nvbio/basic/vector.h>

using namespace nvbio;

namespace nvbio {
namespace cuda {

template <uint32 BLOCKDIM, uint32 VECDIM, uint32 N_TESTS>
__global__ void all_test_kernel(uint32* dst)
{
    __shared__ volatile uint8 sm[ BLOCKDIM >> cuda::Arch::LOG_WARP_SIZE ];

    uint32 r = 0;
    for (uint32 i = 0; i < N_TESTS; ++i)
    {
        const bool p = threadIdx.x > 2*i;
        r += all<VECDIM>( p, sm );
    }
    dst[ threadIdx.x + blockDim.x*blockIdx.x ] = r;
}
template <uint32 BLOCKDIM, uint32 VECDIM, uint32 N_TESTS>
__global__ void any_test_kernel(uint32* dst)
{
    __shared__ volatile uint8 sm[ BLOCKDIM >> cuda::Arch::LOG_WARP_SIZE ];

    uint32 r = 0;
    for (uint32 i = 0; i < N_TESTS; ++i)
    {
        const bool p = threadIdx.x > 2*i;
        r += any<VECDIM>( p, sm );
    }
    dst[ threadIdx.x + blockDim.x*blockIdx.x ] = r;
}

template <uint32 N_THREADS, uint32 BLOCKDIM, uint32 VECDIM, uint32 N_TESTS>
void all_test(uint32* dst)
{
    const uint32 N_RUNS = 16;
    Timer timer;
    timer.start();

    for (uint32 i = 0; i < N_RUNS; ++i)
        all_test_kernel<BLOCKDIM,VECDIM,N_TESTS><<<N_THREADS / BLOCKDIM, BLOCKDIM>>>( dst );

    cudaDeviceSynchronize();

    timer.stop();

    fprintf(stderr, "  all<%u> throughput: %.3f G vectors/s, %.3f G threads/s\n", VECDIM,
        (1.0e-9f * float(N_THREADS/VECDIM)*float(N_TESTS))*(float(N_RUNS)/timer.seconds()),
        (1.0e-9f * float(N_THREADS)*float(N_TESTS))*(float(N_RUNS)/timer.seconds()));
}
template <uint32 N_THREADS, uint32 BLOCKDIM, uint32 VECDIM, uint32 N_TESTS>
void any_test(uint32* dst)
{
    const uint32 N_RUNS = 16;
    Timer timer;
    timer.start();

    for (uint32 i = 0; i < N_RUNS; ++i)
        any_test_kernel<BLOCKDIM,VECDIM,N_TESTS><<<N_THREADS / BLOCKDIM, BLOCKDIM>>>( dst );

    cudaDeviceSynchronize();

    timer.stop();

    fprintf(stderr, "  any<%u> throughput: %.3f G vectors/s, %.3f G threads/s\n", VECDIM,
        (1.0e-9f * float(N_THREADS/VECDIM)*float(N_TESTS))*(float(N_RUNS)/timer.seconds()),
        (1.0e-9f * float(N_THREADS)*float(N_TESTS))*(float(N_RUNS)/timer.seconds()));
}

void scan_test()
{
    const uint32 BLOCKDIM  = 128;
    const uint32 N_THREADS = BLOCKDIM * 1024;
    NVBIO_VAR_UNUSED const uint32 N_TESTS = 128;

    nvbio::vector<device_tag,uint32> dst( N_THREADS );
    uint32* dst_dptr = nvbio::raw_pointer( dst );

    fprintf(stderr, "all test... started\n");
    all_test<N_THREADS,BLOCKDIM,2,N_TESTS>( dst_dptr );
    all_test<N_THREADS,BLOCKDIM,4,N_TESTS>( dst_dptr );
    all_test<N_THREADS,BLOCKDIM,8,N_TESTS>( dst_dptr );
    all_test<N_THREADS,BLOCKDIM,16,N_TESTS>( dst_dptr );
    all_test<N_THREADS,BLOCKDIM,32,N_TESTS>( dst_dptr );
    all_test<N_THREADS,BLOCKDIM,64,N_TESTS>( dst_dptr );
    all_test<N_THREADS,BLOCKDIM,128,N_TESTS>( dst_dptr );
    fprintf(stderr, "all test... done\n");

    fprintf(stderr, "any test... started\n");
    any_test<N_THREADS,BLOCKDIM,2,N_TESTS>( dst_dptr );
    any_test<N_THREADS,BLOCKDIM,4,N_TESTS>( dst_dptr );
    any_test<N_THREADS,BLOCKDIM,8,N_TESTS>( dst_dptr );
    any_test<N_THREADS,BLOCKDIM,16,N_TESTS>( dst_dptr );
    any_test<N_THREADS,BLOCKDIM,32,N_TESTS>( dst_dptr );
    any_test<N_THREADS,BLOCKDIM,64,N_TESTS>( dst_dptr );
    any_test<N_THREADS,BLOCKDIM,128,N_TESTS>( dst_dptr );
    fprintf(stderr, "any test... done\n");
}

} // namespace cuda
} // namespace nvbio
