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

#include <nvbio/basic/numbers.h>

using namespace nvbio;

// divide by two
//
struct divide_by_two
{
    typedef uint32  argument_type;
    typedef uint32  result_type;

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    result_type operator() (const argument_type op) const { return op / 2u; }
};

// return 1 or 0 depending on whether a number is >= than a given threshold
struct above_threshold
{
    typedef int16  argument_type;
    typedef uint32 result_type;

    // constructor
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    above_threshold(const int16 _t) : t(_t) {}

    // functor operator
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 operator() (const int16 s) { return s >= t ? 1u : 0u; }

    const int16 t;
};

// update the best scores vector
//
__global__
void update_scores_kernel(
    const uint32  n,
    const uint32* reads,
    const int16*  scores,
        int16*    best)
{
    const uint32 i = threadIdx.x + blockIdx.x * blockDim.x;
    if (i >= n) return;

    const uint32 read_id = reads[i];
    const int16  score   = scores[i];

    best[ read_id ] = nvbio::max( best[ read_id ], score );
}

// update the best scores vector
//
void update_scores(
    const uint32  n,
    const uint32* reads,
    const int16*  scores,
        int16*    best)
{
    const uint32 block_dim = 128;
    const uint32 n_blocks = util::divide_ri( n, block_dim );

    update_scores_kernel<<<n_blocks,block_dim>>>( n, reads, scores, best );
}
