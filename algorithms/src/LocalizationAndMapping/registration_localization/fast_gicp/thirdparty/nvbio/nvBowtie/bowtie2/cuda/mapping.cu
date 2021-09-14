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

#include <nvBowtie/bowtie2/cuda/mapping.h>
#include <nvBowtie/bowtie2/cuda/mapping_impl.h>

namespace nvbio {
namespace bowtie2 {
namespace cuda {

//
// For all i in [0, #seed hit ranges[, output the seed hit range size in
// out_ranges[i].
//
__global__ 
void gather_ranges_kernel(
    const uint32                        count,
    const uint32                        n_reads,
    const SeedHitDequeArrayDeviceView   hits,
    const uint32*                       hit_counts_scan,
          uint64*                       out_ranges)
{
    const uint32 thread_id = threadIdx.x + BLOCKDIM*blockIdx.x;
    if (thread_id >= count) return;

    // do a binary search, looking for thread_id in hit_counts_scan,
    // to find the corresponding read id.
    const uint32 read_id = upper_bound_index( thread_id, hit_counts_scan, n_reads );

    // at this point we can figure out which seed hit / SA range this thread is
    // responsible of
    const uint32 count_offset = read_id ? hit_counts_scan[read_id-1] : 0u;

    const uint32 range_id = thread_id - count_offset;

    const SeedHit* hits_data = hits.get_data( read_id );

    const uint2 range = hits_data[ range_id ].get_range();
 
    // and we can compute the corresponding range size
    out_ranges[ thread_id ] = range.y - range.x;
}

//
// dispatch the call to gather_ranges_kernel
//
void gather_ranges(
    const uint32                        count,
    const uint32                        n_reads,
    const SeedHitDequeArrayDeviceView   hits,
    const uint32*                       hit_counts_scan,
          uint64*                       out_ranges)
{
    const int blocks = (count + BLOCKDIM-1) / BLOCKDIM;

    gather_ranges_kernel<<<blocks, BLOCKDIM>>>( count, n_reads, hits, hit_counts_scan, out_ranges );
}


//
// perform exact read mapping
//
void map_whole_read(
    const ReadsDef::type&                           read_batch,
    const FMIndexDef::type                          fmi,
    const FMIndexDef::type                          rfmi,
    const nvbio::cuda::PingPongQueuesView<uint32>   queues,
    uint8*                                          reseed,
    SeedHitDequeArrayDeviceView                     hits,
    const ParamsPOD                                 params,
    const bool                                      fw,
    const bool                                      rc)
{
    map_whole_read_t( read_batch, fmi, rfmi, queues, reseed, hits, params, fw, rc );
}

//
// perform one run of exact seed mapping for all the reads in the input queue,
// writing reads that need another run in the output queue
//
void map_exact(
    const ReadsDef::type&                           read_batch,
    const FMIndexDef::type                          fmi,
    const FMIndexDef::type                          rfmi,
    const uint32                                    retry,
    const nvbio::cuda::PingPongQueuesView<uint32>   queues,
    uint8*                                          reseed,
    SeedHitDequeArrayDeviceView                     hits,
    const ParamsPOD                                 params,
    const bool                                      fw,
    const bool                                      rc)
{
    map_exact_t( read_batch, fmi, rfmi, retry, queues, reseed, hits, params, fw, rc );
}

//
// perform multiple runs of exact seed mapping in one go and keep the best
//
void map_exact(
    const ReadsDef::type&                           read_batch,
    const FMIndexDef::type                          fmi,
    const FMIndexDef::type                          rfmi,
    SeedHitDequeArrayDeviceView                     hits,
    const uint2                                     seed_range,
    const ParamsPOD                                 params,
    const bool                                      fw,
    const bool                                      rc)
{
    map_exact_t( read_batch, fmi, rfmi, hits, seed_range, params, fw, rc );
}

//
// perform one run of approximate seed mapping for all the reads in the input queue,
// writing reads that need another run in the output queue
//
void map_approx(
    const ReadsDef::type&                           read_batch,
    const FMIndexDef::type                          fmi,
    const FMIndexDef::type                          rfmi,
    const uint32                                    retry,
    const nvbio::cuda::PingPongQueuesView<uint32>   queues,
    uint8*                                          reseed,
    SeedHitDequeArrayDeviceView                     hits,
    const ParamsPOD                                 params,
    const bool                                      fw,
    const bool                                      rc)
{
    map_approx_t( read_batch, fmi, rfmi, retry, queues, reseed, hits, params, fw, rc );
}

//
// perform multiple runs of approximate seed mapping in one go and keep the best
//
void map_approx(
    const ReadsDef::type&                           read_batch,
    const FMIndexDef::type                          fmi,
    const FMIndexDef::type                          rfmi,
    SeedHitDequeArrayDeviceView                     hits,
    const uint2                                     seed_range,
    const ParamsPOD                                 params,
    const bool                                      fw,
    const bool                                      rc)
{
    map_approx_t( read_batch, fmi, rfmi, hits, seed_range, params, fw, rc );
}

//
// perform one run of seed mapping
//
void map(
    const ReadsDef::type&                           read_batch,
    const FMIndexDef::type                          fmi,
    const FMIndexDef::type                          rfmi,
    const uint32                                    retry,
    const nvbio::cuda::PingPongQueuesView<uint32>   queues,
    uint8*                                          reseed,
    SeedHitDequeArrayDeviceView                     hits,
    const ParamsPOD                                 params,
    const bool                                      fw,
    const bool                                      rc)
{
    map_t( read_batch, fmi, rfmi, retry, queues, reseed, hits, params, fw, rc );
}

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
