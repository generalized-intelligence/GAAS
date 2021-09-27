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

#include <nvBowtie/bowtie2/cuda/checksums.h>
#include <nvBowtie/bowtie2/cuda/mapping.h>
#include <nvBowtie/bowtie2/cuda/seed_hit.h>
#include <crc/crc.h>
#include <thrust/scan.h>

namespace nvbio {
namespace bowtie2 {
namespace cuda {

// compute a checksum for the returned SA ranges
//
void hits_checksum(
    const uint32                n_reads,
          SeedHitDequeArray&    hit_deques,
          uint64&               crc,
          uint64&               sum)
{
    thrust::device_vector<uint32> hits_count_scan_dvec( n_reads );

    // run a scan on the number of SA ranges from each read
    thrust::inclusive_scan( hit_deques.counts().begin(), hit_deques.counts().begin() + n_reads, hits_count_scan_dvec.begin() );

    // compute how many ranges we have
    const uint32 n_hit_ranges = hits_count_scan_dvec[ n_reads-1 ];

    // gather all the range sizes, sorted by read, in a compacted array
    thrust::device_vector<uint64> hits_range_scan_dvec( n_hit_ranges );

    SeedHitDequeArrayDeviceView hits = hit_deques.device_view();

    gather_ranges(
        n_hit_ranges,
        n_reads,
        hits,
        thrust::raw_pointer_cast( &hits_count_scan_dvec.front() ),
        thrust::raw_pointer_cast( &hits_range_scan_dvec.front() ) );

    // compute the crc
    crc = device_checksum(
        hits_range_scan_dvec.begin(),
        hits_range_scan_dvec.begin() + n_hit_ranges );

    // scan the ranges
    thrust::inclusive_scan( hits_range_scan_dvec.begin(), hits_range_scan_dvec.begin() + n_hit_ranges, hits_range_scan_dvec.begin() );

    // fetch the total
    sum = hits_range_scan_dvec[ n_hit_ranges - 1 ];
}

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
