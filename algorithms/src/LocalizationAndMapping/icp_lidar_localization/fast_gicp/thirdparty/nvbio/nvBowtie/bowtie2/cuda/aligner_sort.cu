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

#include <nvBowtie/bowtie2/cuda/aligner.h>
#include <nvbio/basic/cuda/sort.h>

namespace nvbio {
namespace bowtie2 {
namespace cuda {

// return a pointer to an "index" into the given keys sorted by their hi bits
//
uint32* Aligner::sort_hi_bits(
    const uint32    count,
    const uint32*   keys)
{
    thrust::transform(
        thrust::device_ptr<const uint32>( keys ),
        thrust::device_ptr<const uint32>( keys ) + count,
        sorting_queue_dvec.begin(),
        hi_bits_functor<uint16,uint32>() );

    thrust::copy(
        thrust::make_counting_iterator(0u),
        thrust::make_counting_iterator(0u) + count,
        idx_queue_dvec.begin() );

    // Create ping-pong storage wrapper
    nvbio::cuda::SortBuffers<uint16*,uint32*> double_buffer;
    double_buffer.keys[0]   = thrust::raw_pointer_cast( &sorting_queue_dvec.front() );
    double_buffer.keys[1]   = thrust::raw_pointer_cast( &sorting_queue_dvec.front() + BATCH_SIZE );
    double_buffer.values[0] = idx_queue_dptr;
    double_buffer.values[1] = idx_queue_dptr + BATCH_SIZE;

    sort_enactor.sort( count, double_buffer );

    return double_buffer.values[double_buffer.selector];
}

// return a pointer to an "index" into the given sorted keys
//
std::pair<uint32*,uint64*> Aligner::sort_64_bits(
    const uint32 count)
{
    thrust::copy(
        thrust::make_counting_iterator(0u),
        thrust::make_counting_iterator(0u) + count,
        idx_queue_dvec.begin() );

    // Create ping-pong storage wrapper
    nvbio::cuda::SortBuffers<uint64*,uint32*> double_buffer;
    double_buffer.keys[0]   = reinterpret_cast<uint64*>( raw_pointer( sorting_queue_dvec ) );
    double_buffer.keys[1]   = reinterpret_cast<uint64*>( raw_pointer( sorting_queue_dvec ) ) + BATCH_SIZE;
    double_buffer.values[0] = idx_queue_dptr;
    double_buffer.values[1] = idx_queue_dptr + BATCH_SIZE;

    sort_enactor.sort( count, double_buffer );

    return std::make_pair(
        double_buffer.values[double_buffer.selector],
        double_buffer.keys[double_buffer.selector] );
}

// sort a set of keys in place
//
void Aligner::sort_inplace(
    const uint32    count,
    uint32*         keys)
{
    // create the ping-pong storage wrapper
    nvbio::cuda::SortBuffers<uint32*> double_buffer;
    double_buffer.keys[0]   = keys;
    double_buffer.keys[1]   = (uint32*)thrust::raw_pointer_cast( &sorting_queue_dvec.front() );

    // enact the sort
    sort_enactor.sort( count, double_buffer );

    // copy the sorted data back in place if it ended up in the temporary buffer
    if (double_buffer.selector)
        cudaMemcpy( keys, double_buffer.keys[1], count * sizeof(uint32), cudaMemcpyDeviceToDevice );
}

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio