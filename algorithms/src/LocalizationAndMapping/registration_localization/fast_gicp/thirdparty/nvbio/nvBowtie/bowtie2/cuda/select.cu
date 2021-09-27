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

#include <nvBowtie/bowtie2/cuda/select.h>
#include <nvBowtie/bowtie2/cuda/select_impl.h>
#include <nvbio/basic/algorithms.h>

namespace nvbio {
namespace bowtie2 {
namespace cuda {

__global__
void select_init_kernel(
    const uint32                            count,
    const char*                             read_names,
    const uint32*                           read_names_idx,
    SeedHitDequeArrayDeviceView             hits,
    uint32*                                 trys,
    uint32*                                 rseeds,
    const ParamsPOD                         params)
{
    //
    // NOTE: here we are initializing constants for ALL the reads in a batch,
    // including inactive ones. This works because we always access these fields
    // by read id throughout the entire pipeline.
    //

    const uint32 thread_id = threadIdx.x + BLOCKDIM*blockIdx.x;
    if (thread_id >= count) return;

    // initialize the number of trys
    if (trys)
        trys[ thread_id ] = params.max_effort_init;

    // initialize the probability trees
    if (params.randomized)
    {
        // initialize the generator
        //rseeds[ thread_id ] = 0u;
        {
            // compute a hash of the read name
            const uint32 off = read_names_idx[ thread_id ];
            const uint32 len = read_names_idx[ thread_id + 1u ] - off;

            const char* read_name = read_names + off;

            // djb2 hashing function
            uint32 hash = 5381;
            for (uint32 i = 0; i < len && read_name[i]; ++i)
                hash = ((hash << 5) + hash) ^ read_name[i];

            rseeds[ thread_id ] = hash;
        }

        typedef SumTree<float*> ProbTree;

        // check whether we have a non-zero number of hits, otherwise we can go home
        const uint32 n_hits = hits.get_size( thread_id );
        if (n_hits == 0u)
            return;

        // build the probability tree
        float*         hit_probs = hits.get_probs( thread_id );
        const SeedHit* hit_data  = hits.get_data( thread_id );

        for (uint32 i = 0; i < n_hits; ++i)
        {
            const SeedHit hit = hit_data[i];
            const uint2 range = hit.get_range();
            hit_probs[i] = 1.0f / (float(range.y - range.x) *
                                   float(range.y - range.x));
        }
        if (params.top_seed)
            hit_probs[0] = 0.0f; // assign zero probability to the top hit, as we will deterministically visit it

        // setup the tree
        ProbTree prob_tree( nvbio::max( n_hits, 1u ), hit_probs );
        prob_tree.setup();
    }
}

//
// Initialize the hit-selection pipeline
//
void select_init(
    const uint32                    count,
    const char*                     read_names,
    const uint32*                   read_names_idx,
    SeedHitDequeArrayDeviceView     hits,
    uint32*                         trys,
    uint32*                         rseeds,
    const ParamsPOD                 params)
{
    const int blocks = (count + BLOCKDIM-1) / BLOCKDIM;

    select_init_kernel<<<blocks, BLOCKDIM>>>(
        count,
        read_names,
        read_names_idx,
        hits,
        trys,
        rseeds,
        params );
}

//
// Initialize the hit-selection pipeline
//
void select_init(BestApproxScoringPipelineState<EditDistanceScoringScheme>& pipeline, const ParamsPOD& params)
{
    select_init_t( pipeline, params );
}

//
// Initialize the hit-selection pipeline
//
void select_init(BestApproxScoringPipelineState<SmithWatermanScoringScheme<> >& pipeline, const ParamsPOD& params)
{
    select_init_t( pipeline, params );
}

//
// Prepare for a round of seed extension by selecting the next SA row from each
// of the seed-hit deque arrays (SeedHitDequeArray) bound to the active-reads in
// the scoring queues (ScoringQueues::active_reads).
//
void select(
    const SelectBestApproxContext                                       context,
    const BestApproxScoringPipelineState<EditDistanceScoringScheme>&    pipeline,
    const ParamsPOD                                                     params)
{
    select_t( context, pipeline, params );
}

//
// Prepare for a round of seed extension by selecting the next SA row from each
// of the seed-hit deque arrays (SeedHitDequeArray) bound to the active-reads in
// the scoring queues (ScoringQueues::active_reads).
//
void select(
    const SelectBestApproxContext                                           context,
    const BestApproxScoringPipelineState<SmithWatermanScoringScheme<> >&    pipeline,
    const ParamsPOD                                                         params)
{
    select_t( context, pipeline, params );
}

///
/// select next hit extensions for all-mapping
///
__global__ 
void select_all_kernel(
    const uint64                    begin,
    const uint32                    count,
    const uint32                    n_reads,
    const uint32                    n_hit_ranges,
    const uint64                    n_hits,
    SeedHitDequeArrayDeviceView     hits,
    const uint32*                   hit_count_scan,
    const uint64*                   hit_range_scan,
          HitQueuesDeviceView       hit_queues)
{
    const uint32 thread_id = threadIdx.x + BLOCKDIM*blockIdx.x;
    if (thread_id >= count) return;

    // each thread is responsible to process a hit, starting from 'begin'
    const uint64 global_hit_id = begin + thread_id;

    // do a binary search, looking for thread_id in hit_range_scan,
    // to find the corresponding seed hit.
    const uint32 global_range_id = uint32( upper_bound( global_hit_id, hit_range_scan, n_hit_ranges ) - hit_range_scan );

    // at this point we can figure out which read this range belongs to
    const uint32 read_id = uint32( upper_bound( global_range_id, hit_count_scan, n_reads ) - hit_count_scan );

    // now we have everything we need to access the proper hit_data
    const SeedHit* hit_data = hits.get_data( read_id );

    // compute the local range index within this read
    const uint32 count_offset = read_id ? hit_count_scan[ read_id-1 ] : 0u;
    const uint32 range_id = global_range_id - count_offset;

    const SeedHit* hit = &hit_data[ range_id ];

    // compute the local hit index within this range
    const uint64 range_offset = global_range_id ? hit_range_scan[ global_range_id-1 ] : 0u;
    const uint32 hit_id = uint32( global_hit_id - range_offset );

    const uint32 r_type = hit->get_readtype() ? 1u : 0u;

    HitReference<HitQueuesDeviceView> out_hit( hit_queues, thread_id );
    out_hit.loc     = hit->front() + hit_id;
    out_hit.seed    = packed_seed( hit->get_posinread(), hit->get_indexdir(), r_type, 0u );
    out_hit.read_id = read_id;
}

///
/// select next hit extensions from the top seed ranges
///
__global__ 
void select_n_from_top_range_kernel(
    const uint32                    begin,
    const uint32                    count,
    const uint32                    n_reads,
    SeedHitDequeArrayDeviceView     hits,
    const uint32*                   hit_range_scan,
          uint32*                   loc_queue,
          packed_seed*              seed_queue,
          packed_read*              read_info)
{
    const uint32 thread_id = threadIdx.x + BLOCKDIM*blockIdx.x;
    if (thread_id >= count) return;

    // each thread is responsible to process a hit, starting from 'begin'
    const uint64 global_hit_id = begin + thread_id;

    // do a binary search, looking for thread_id in hit_range_scan,
    // to find the corresponding read id.
    const uint32 read_id = uint32( upper_bound( global_hit_id, hit_range_scan, n_reads ) - hit_range_scan );

    // now we have everything we need to access the proper hit_data
    const SeedHit* hit_data = hits.get_data( read_id );

    // fetch the top range
    const SeedHit* hit = &hit_data[0];

    // compute the local hit index within this range
    const uint64 range_offset = global_hit_id ? hit_range_scan[ read_id-1 ] : 0u;
    const uint32 hit_id = uint32( global_hit_id - range_offset );

    const uint32 r_type = hit->get_readtype() ? 1u : 0u;

    loc_queue[ thread_id ]  = hit->front() + hit_id;
    seed_queue[ thread_id ] = packed_seed( hit->get_posinread(), hit->get_indexdir(), r_type, 0u );
    read_info[ thread_id ]  = packed_read( read_id );
}

void select_all(
    const uint64                        begin,
    const uint32                        count,
    const uint32                        n_reads,
    const uint32                        n_hit_ranges,
    const uint64                        n_hits,
    const SeedHitDequeArrayDeviceView   hits,
    const uint32*                       hit_count_scan,
    const uint64*                       hit_range_scan,
          HitQueuesDeviceView           hit_queues)
{
    const int blocks = (count + BLOCKDIM-1) / BLOCKDIM;

    select_all_kernel<<<blocks, BLOCKDIM>>>(
        begin,
        count,
        n_reads,
        n_hit_ranges,
        n_hits,
        hits,
        hit_count_scan,
        hit_range_scan,
        hit_queues );
}

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
