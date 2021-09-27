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

//#define NVBIO_CUDA_DEBUG
//#define NVBIO_CUDA_ASSERTS

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <nvbio/basic/timer.h>
#include <nvbio/basic/console.h>
#include <nvBowtie/bowtie2/cuda/seed_hit_deque_array.h>


namespace nvbio {
namespace bowtie2 {
namespace cuda {

namespace { // anonymous namespace

__global__
void setup_deques_kernel(SeedHitDequeArrayDeviceView seed_hit_deques, const uint32 n_reads, uint32* error)
{
    if (threadIdx.x >= n_reads)
        return;

    typedef SeedHitDequeArrayDeviceView::reference      hit_deque_reference;
    typedef SeedHitDequeArrayDeviceView::hit_deque_type hit_deque_type;

    // fetch the deque bound to this read
    hit_deque_reference hit_deque = seed_hit_deques[ threadIdx.x ];

    // alloc storage for 2 entries in this read's deque
    hit_deque.alloc( 2u );

    // push first hit
    hit_deque.push( SeedHit( STANDARD, FORWARD, threadIdx.x * 2u + 1u, make_uint2( 0, 100 ) ) );

    // push second hit
    hit_deque.push( SeedHit( STANDARD, FORWARD, threadIdx.x * 2u + 0u, make_uint2( 0, 10 ) ) );
}

__global__
void check_deques_kernel(SeedHitDequeArrayDeviceView seed_hit_deques, const uint32 n_reads, uint32* error)
{
    if (threadIdx.x >= n_reads)
        return;

    typedef SeedHitDequeArrayDeviceView::reference      hit_deque_reference;
    typedef SeedHitDequeArrayDeviceView::hit_deque_type hit_deque_type;

    // fetch the deque bound to this read
    hit_deque_reference hit_deque = seed_hit_deques[ threadIdx.x ];

    SeedHit hit;

    // pop first hit
    hit = hit_deque.top(); hit_deque.pop();
    if (hit.get_posinread() != threadIdx.x * 2u + 0u)
        *error = 1;

    // pop second hit
    hit = hit_deque.top(); hit_deque.pop();
    if (hit.get_posinread() != threadIdx.x * 2u + 1u)
        *error = 2;
}

} // anonymous namespace

void test_seed_hit_deques()
{
    log_info(stderr, "test seed_hit_deques... started\n");
    SeedHitDequeArray seed_hit_deques;

    const uint32 n_hits          = 100;
    const uint32 n_reads         = 50;

    const uint64 bytes = seed_hit_deques.resize( n_reads, n_hits );
    log_info(stderr, "  allocated %llu bytes\n", bytes);

    thrust::device_vector<uint32> error(1,0);

    setup_deques_kernel<<<1,128>>>( seed_hit_deques.device_view(), n_reads, device_view( error ) );
    check_deques_kernel<<<1,128>>>( seed_hit_deques.device_view(), n_reads, device_view( error ) );
    cudaThreadSynchronize();

    const uint32 error_code = error[0];
    if (error_code)
        log_error( stderr, "test_read_hits_index failed! (error code: %u)\n", error_code );

    log_info(stderr, "test seed_hit_deques... done\n");
}

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
