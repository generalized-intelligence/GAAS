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

#include "filter-chains.h"
#include "mem-search.h"
#include "options.h"
#include "pipeline.h"
#include "util.h"

#include <nvbio/basic/numbers.h>
#include <nvbio/basic/algorithms.h>
#include <nvbio/basic/priority_queue.h>
#include <nvbio/basic/timer.h>
#include <nvbio/basic/transform_iterator.h>
#include <nvbio/basic/vector_view.h>
#include <nvbio/basic/primitives.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/iterator/transform_iterator.h>
#include <thrust/sort.h>

using namespace nvbio;

// compute the coverage for each chain in a set
__global__
void chain_coverage_kernel(
    const uint32                                    n_chains,           // the number of chains
    const uint32*                                   chain_reads,        // the chain reads
    const uint32*                                   chain_offsets,      // the chain offsets
    const uint32*                                   chain_lengths,      // the chain lengths
    const mem_state::mem_type*                      mems,               // the MEMs for this chunk of reads
    const uint32*                                   mems_index,         // a sorting index into the MEMs specifying their processing order
          uint2*                                    chain_ranges,       // the output chain ranges
          uint64*                                   chain_weights)      // the output chain weights
{
    const uint32 chain_id = threadIdx.x + blockIdx.x * blockDim.x;
    if (chain_id >= n_chains)
        return;

    const uint32 read  = chain_reads[ chain_id ];
    const uint32 begin = chain_offsets[ chain_id ];
    const uint32 end   = chain_lengths[ chain_id ] + begin;

    uint2  range  = make_uint2( uint32(-1), 0u );
    uint32 weight = 0;

    // NOTE: we assume here the MEMs of a chain appear sorted by their left coordinate
    for (uint32 i = begin; i < end; ++i)
    {
        const mem_state::mem_type seed = mems[ mems_index[i] ];

        const uint2 span = seed.span();

        if (span.x >= range.y)
            weight += span.y - span.x;
        else if (span.y > range.y)
            weight += span.y - range.y;

        range.x = nvbio::min( range.x, seed.span().x );
        range.y = nvbio::max( range.y, seed.span().y );
    }

    // write out the outputs
    chain_ranges[ chain_id ]  = range;
    chain_weights[ chain_id ] = uint64( weight ) | (uint64( read ) << 32);
}

// filter the chains belonging to each read
__global__
void chain_filter_kernel(
    const read_chunk                                chunk,              // the current sub-batch
    const uint32                                    n_chains,           // the number of chains
    const uint32*                                   chain_reads,        // the chain reads
    const uint32*                                   chain_index,        // the chain order
    const uint2*                                    chain_ranges,       // the chain ranges
    const uint64*                                   chain_weights,      // the chain weights
    const float                                     mask_level,         // input option
    const float                                     chain_drop_ratio,   // input option
    const uint32                                    min_seed_len,       // input option
          uint8*                                    chain_flags)        // the output flags
{
    const uint32 read_id = threadIdx.x + blockIdx.x * blockDim.x + chunk.read_begin;
    if (read_id >= chunk.read_end)
        return;

    const uint32 begin = uint32( nvbio::lower_bound( read_id, chain_reads, n_chains ) - chain_reads );
    const uint32 end   = uint32( nvbio::upper_bound( read_id, chain_reads, n_chains ) - chain_reads );

    // skip pathological cases
    if (begin == end)
        return;

    // keep the first chain
    chain_flags[ chain_index[begin] ] = 1u; // mark to keep

    // and loop through all the rest to decide which ones to keep
    uint32 n = 1;

    for (uint32 i = begin + 1; i < end; ++i)
    {
        const uint2  i_span = chain_ranges[ chain_index[i] ];
        const uint32 i_w    = chain_weights[ i ] & 0xFFFFFFFFu;               // already sorted as chain_index

        uint32 j;
        for (j = begin; j < begin + n; ++j)
        {
            const uint2  j_span = chain_ranges[ chain_index[j] ];
            const uint32 j_w    = chain_weights[ j ] & 0xFFFFFFFFu;           // already sorted as chain_index

            const uint32 max_begin = nvbio::max( i_span.x, j_span.x );
            const uint32 min_end   = nvbio::min( i_span.y, j_span.y );

            if (min_end > max_begin) // have overlap
            {
                const uint32 min_l = nvbio::min( i_span.y - i_span.x, j_span.y - j_span.x );
				if (min_end - max_begin >= min_l * mask_level) // significant overlap
                {
                    chain_flags[ chain_index[i] ] = 1u; // mark to keep

                    if (i_w < j_w * chain_drop_ratio &&
                        j_w - i_w >= min_seed_len * 2)
                        break;
				}
            }
        }
		if (j == n) // no significant overlap with better chains, keep it.
        {
            chain_flags[ chain_index[i] ] = 1u; // mark to keep

            ++n;
        }
    }
}

// filter chains for the current pipeline::chunk of reads
void filter_chains(pipeline_state *pipeline, const io::SequenceDataDevice *reads)
{
    const ScopedTimer<float> timer( &pipeline->stats.chain_time ); // keep track of the time spent here

    struct chains_state<device_tag> *chn = &pipeline->chn;

    const uint32 n_reads = pipeline->chunk.read_end - pipeline->chunk.read_begin;
    const uint32 n_mems  = pipeline->chunk.mem_end  - pipeline->chunk.mem_begin;

    // skip pathological cases
    if (n_mems == 0u)
        return;

    // extract the list of unique chain ids together with their counts, i.e. the chain lengths
    nvbio::vector<device_tag,uint64> unique_chains( n_mems );
    nvbio::vector<device_tag,uint32> unique_counts( n_mems );
    nvbio::vector<device_tag,uint8>  temp_storage;

    const uint32 n_chains = runlength_encode(
        n_mems,
        chn->mems_chain.begin(),                        // the input chain ids, one per seed
        unique_chains.begin(),                          // the output "unique" chain ids
        unique_counts.begin(),                          // the output repetition counts, i.e. the chain lengths
        temp_storage );                                 // some temp storage

    // resize the chain vectors if needed
    uint32 reserved_space = uint32( chn->chain_lengths.size() );
    if (n_chains > reserved_space)
    {
        chn->chain_lengths.clear();  chn->chain_lengths.resize( n_chains );
        chn->chain_offsets.clear();  chn->chain_offsets.resize( n_chains );
        chn->chain_reads.clear();    chn->chain_reads.resize( n_chains );

        reserved_space = n_chains;
    }

    // copy their lengths
    thrust::copy(
        unique_counts.begin(),
        unique_counts.begin() + n_chains,
        chn->chain_lengths.begin() );

    // find the offset to the beginning of each chain
    thrust::lower_bound(
        chn->mems_chain.begin(),                    // the beginning of the sorted list of keys to search in
        chn->mems_chain.begin() + n_mems,           // the end of the sorted list of keys to search in
        unique_chains.begin(),                      // the beginning of the sequence of values to search
        unique_chains.begin() + n_chains,           // the end of the sequence of values to search
        chn->chain_offsets.begin() );               // the output sequence

    // extract the read-id frome the chain ids
    thrust::transform(
        unique_chains.begin(),                      // the beginning of the input sequence to transform
        unique_chains.begin() + n_chains,           // the end of the input sequence to transform
        chn->chain_reads.begin(),                   // the beginning othe output sequence
        nvbio::hi_bits_functor<uint32,uint64>() );  // the functor to apply, in this case a 32-bit left shift

    // debug check: make sure the chain offsets are sorted
    if (is_sorted<device_tag>( n_chains, chn->chain_offsets.begin() ) == false)
    {
        log_error(stderr, "filter_chains: chain offsets are not sorted!\n");
        exit(0);
    }

    // debug check: make sure the chains are sorted by read
    if (is_sorted<device_tag>( n_chains, chn->chain_reads.begin() ) == false)
    {
        log_error(stderr, "filter_chains: chains are not sorted by read!\n");
        exit(0);
    }

    nvbio::vector<device_tag,uint2>  chain_ranges( n_chains );
    nvbio::vector<device_tag,uint64> chain_weights( n_chains );
    nvbio::vector<device_tag,uint32> chain_index( reserved_space ); // potentially a little bigger because we'll reuse
                                                                    // it for the final filtering...

    optional_device_synchronize();
    cuda::check_error("chain-coverage-init");

    // compute chain coverages
    {
        const uint32 block_dim = 128;
        const uint32 n_blocks  = util::divide_ri( n_chains, block_dim );

        chain_coverage_kernel<<<n_blocks, block_dim>>>(
            n_chains,
            nvbio::plain_view( chn->chain_reads ),
            nvbio::plain_view( chn->chain_offsets ),
            nvbio::plain_view( chn->chain_lengths ),
            nvbio::plain_view( chn->mems ),
            nvbio::plain_view( chn->mems_index ),
            nvbio::plain_view( chain_ranges ),
            nvbio::plain_view( chain_weights ) );

        optional_device_synchronize();
        cuda::check_error("chain-coverage kernel");
    }

    // sort the chains by weight
    thrust::copy(
        thrust::make_counting_iterator<uint32>(0u),
        thrust::make_counting_iterator<uint32>(0u) + n_chains,
        chain_index.begin() );

    thrust::sort_by_key(                            // TODO: this is slow, switch to nvbio::cuda::SortEnactor
        chain_weights.begin(),
        chain_weights.begin() + n_chains,
        chain_index.begin() );

    nvbio::vector<device_tag,uint8> chain_flags( n_chains );
    thrust::fill( chain_flags.begin(), chain_flags.begin() + n_chains, 0u );

    // filter chains: set the flags for the chains to be kept
    {
        const uint32 block_dim = 128;
        const uint32 n_blocks  = util::divide_ri( n_reads, block_dim );

        chain_filter_kernel<<<n_blocks, block_dim>>>(
            pipeline->chunk,
            n_chains,
            nvbio::plain_view( chn->chain_reads ),
            nvbio::plain_view( chain_index ),
            nvbio::plain_view( chain_ranges ),
            nvbio::plain_view( chain_weights ),
            command_line_options.mask_level,
            command_line_options.chain_drop_ratio,
            command_line_options.min_seed_len,
            nvbio::plain_view( chain_flags ) );

        optional_device_synchronize();
        cuda::check_error("chain-filter kernel");
    }

    // filter chain_reads
    const uint32 n_filtered_chains = copy_flagged(
        n_chains,                                   // the number of input elements
        chn->chain_reads.begin(),                   // the input sequence of flagged elements to copy
        chain_flags.begin(),                        // the input sequence of flags
        chain_index.begin(),                        // the output sequence of copied elements
        temp_storage );                             // some temporary storage

    chn->chain_reads.swap( chain_index );

    // debug check: make sure the chains are sorted by read
    if (is_sorted<device_tag>( n_filtered_chains, chn->chain_reads.begin() ) == false)
    {
        log_error(stderr, "filter_chains: filtered chains are not sorted by read!\n");
        exit(0);
    }

    // filter chain_offsets
    cuda::copy_flagged(
        n_chains,                                   // the number of input elements
        chn->chain_offsets.begin(),                 // the input sequence of flagged elements to copy
        chain_flags.begin(),                        // the input sequence of flags
        chain_index.begin(),                        // the output sequence of copied elements
        temp_storage );                             // some temporary storage

    chn->chain_offsets.swap( chain_index );

    // filter chain_lengths
    cuda::copy_flagged(
        n_chains,                                   // the number of input elements
        chn->chain_lengths.begin(),                 // the input sequence of flagged elements to copy
        chain_flags.begin(),                        // the input sequence of flags
        chain_index.begin(),                        // the output sequence of copied elements
        temp_storage );                             // some temporary storage

    chn->chain_lengths.swap( chain_index );

    // assign the output number of chains
    chn->n_chains = n_filtered_chains;

    // keep stats
    pipeline->stats.n_chains += n_filtered_chains;
}
