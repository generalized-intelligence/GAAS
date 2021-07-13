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

#include <nvBowtie/bowtie2/cuda/defs.h>
#include <nvBowtie/bowtie2/cuda/seed_hit_deque_array.h>
#include <nvbio/basic/types.h>
#include <crc/crc.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>

namespace nvbio {
namespace bowtie2 {
namespace cuda {

struct SeedHit;

// compute a checksum of a device sequence
//
template <typename Iterator>
uint64 device_checksum(
    Iterator    begin,
    Iterator    end)
{
    typedef typename thrust::iterator_traits<Iterator>::value_type value_type;

    const uint32 n = uint32( end - begin );

    // copy values to a temporary vector
    thrust::device_vector<value_type> debug_copy_dvec( n );
    thrust::copy(
        begin,
        end,
        debug_copy_dvec.begin() );

    // sort temporary vector
    thrust::sort( debug_copy_dvec.begin(), debug_copy_dvec.end() );

    // copy to the host
    thrust::host_vector<value_type> debug_copy_hvec( debug_copy_dvec );

    // compute a crc
    const char* ptr = (const char*)thrust::raw_pointer_cast( &debug_copy_hvec.front() );
    return crcCalc( ptr, sizeof(value_type)*n );
}

// compute a checksum for the returned SA ranges
//
void hits_checksum(
    const uint32                n_reads,
          SeedHitDequeArray&    hit_deques,
          uint64&               crc,
          uint64&               sum);

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
