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

#include "build-chains.h"
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

// a functor to extract the read id from a mem
struct mem_read_id_functor
{
    typedef mem_state::mem_type argument_type;
    typedef uint32              result_type;

    NVBIO_HOST_DEVICE
    uint32 operator() (const argument_type mem) const { return mem.string_id(); }
};

// a class to keep track of a chain
struct chain
{
    // construct an empty chain
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    chain() : id(uint32(-1)) {}

    // construct a new chain from a single seed
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    chain(const uint32 _id, const mem_state::mem_type seed) :
        id( _id ),
        ref( seed.index_pos() ),
        span_beg( seed.span().x ),
        last_ref( seed.index_pos() ),
        last_span( seed.span() )
    {}

    // test whether we can merge the given mem into this chain
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    bool merge(const mem_state::mem_type seed, const uint32 w, const uint32 max_chain_gap)
    {
        const uint32 seed_len = seed.span().y - seed.span().x;
        const uint32 last_len = last_span.y - last_span.x;
        const uint32 rbeg     = ref;
        const uint32 rend     = last_ref + last_len;

        // check whether seed is contained in the chain
        if (seed.span().x >= span_beg && seed.span().y <= last_span.y && seed.index_pos() >= rbeg && seed.index_pos() + seed_len <= rend)
            return true; // contained seed; do nothing

    	const int32 x = seed.span().x - last_span.x; // always non-negative
        const int32 y = seed.index_pos() - last_ref;
        if ((y >= 0) && (x - y <= w) && (x - last_len < max_chain_gap) && (y - last_len < max_chain_gap))
        {
             // grow the chain
            last_span = seed.span();
            last_ref  = seed.index_pos();
            return true;
        }
        return false;
    }

    uint32 id;              // chain id
    uint32 ref;             // reference coordinate of the first seed in the chain
    uint32 span_beg;        // read span begin
    uint32 last_ref;        // the reference coordinate of the last seed in the chain
    uint2  last_span;       // the read span of the last seed in the chain
};

struct chain_compare
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    bool operator() (const chain& chain1, const chain& chain2) const
    {
        // compare by the reference coordinate of the first seed of each chain
        return chain1.ref < chain2.ref;
    }
};

// assign a chain id to all MEMs for the current pipeline::chunk of reads
__global__
void build_chains_kernel(
    const read_chunk                                            chunk,              // the current sub-batch
    const uint32                                                pass_number,        // the pass number - we process up to N seeds per pass
    const uint32                                                n_active,           // the number of active reads in this pass
    const uint32*                                               active_reads,       // the set of active reads
          uint8*                                                active_flags,       // the output set of active read flags
    const uint32                                                w,                  // w parameter
    const uint32                                                max_chain_gap,      // max chain gap parameter
    const uint32                                                n_mems,             // the total number of MEMs for this chunk of reads
    const mem_state::mem_type*                                  mems,               // the MEMs for this chunk of reads
    const uint32*                                               mems_index,         // a sorting index into the MEMs specifying the processing order
          uint64*                                               mems_chains)        // the output chain IDs corresponding to the sorted MEMs
{
    const uint32 thread_id = threadIdx.x + blockIdx.x * blockDim.x;
    if (thread_id >= n_active)
        return;

    const uint32 read_id = active_reads[ thread_id ];

    // find the first seed belonging to this read
    const uint32 mem_begin = uint32( nvbio::lower_bound(
        read_id,
        nvbio::make_transform_iterator( mems, mem_read_id_functor() ),
        n_mems ) - nvbio::make_transform_iterator( mems, mem_read_id_functor() ) );

    // find the first seed belonging to the next read
    const uint32 mem_end = uint32( nvbio::lower_bound(
        read_id+1u,
        nvbio::make_transform_iterator( mems, mem_read_id_functor() ),
        n_mems ) - nvbio::make_transform_iterator( mems, mem_read_id_functor() ) );

    // the maximum amount of chains we can output in one pass
    const uint32 MAX_CHAINS = 128;

    // keep a priority queue of the chains organized by the reference coordinate of their leftmost seed
    typedef nvbio::vector_view<chain*>                                          chain_vector_type;
    typedef nvbio::priority_queue<chain, chain_vector_type, chain_compare>      chain_queue_type;

    chain            chain_queue_storage[MAX_CHAINS+1];
    chain_queue_type chain_queue( chain_vector_type( 0u, chain_queue_storage ) );

    // keep a counter tracking the number of chains that get created
    //
    // NOTE: here we conservatively assume that in the previous passes we have
    // created the maximum number of chains, so as to avoid assigning an already
    // taken ID to a new chain (which would result in merging potentially unrelated
    // chains)
    uint64 n_chains = pass_number * MAX_CHAINS;

    // compute the first and ending MEM to process in this pass
    const uint32 mem_batch_begin = mem_begin + pass_number * MAX_CHAINS;
    const uint32 mem_batch_end   = nvbio::min( mem_batch_begin + MAX_CHAINS, mem_end );

    // process the seeds in order
    for (uint32 i = mem_batch_begin; i < mem_batch_end; ++i)
    {
        const uint32 seed_idx          = mems_index[i];
        const mem_state::mem_type seed = mems[ seed_idx ];

        // the chain id for this seed, to be determined
        uint32 chain_id;

        // insert seed
        if (chain_queue.empty())
        {
            // get a new chain id
            chain_id = n_chains++;

            // build a new chain
            chain_queue.push( chain( chain_id, seed ) );
        }
        else
        {
            // find the closest chain...
            chain_queue_type::iterator chain_it = chain_queue.upper_bound( chain( 0u, seed ) );

            // and test whether we can merge this seed into it
            if (chain_it != chain_queue.end() &&
                chain_it->merge( seed, w, max_chain_gap ) == false)
            {
                // get a new chain id
                chain_id = n_chains++;

                // build a new chain
                chain_queue.push( chain( chain_id, seed ) );
            }
            else
            {
                // merge with the existing chain
                chain_id = chain_it->id;
            }
        }

        // write out the chain id (OR'd with the read id)
        mems_chains[i] = chain_id | (uint64( read_id ) << 32);
    }

    // write out whether we need more passes
    active_flags[ thread_id ] = (mem_batch_begin < mem_end) ? 1u : 0u;
}

// build chains for the current pipeline::chunk of reads
void build_chains(pipeline_state *pipeline, const io::SequenceDataDevice *reads)
{
    const ScopedTimer<float> timer( &pipeline->stats.chain_time ); // keep track of the time spent here

    struct chains_state<device_tag> *chn = &pipeline->chn;

    const uint32 n_reads = pipeline->chunk.read_end - pipeline->chunk.read_begin;
    const uint32 n_mems  = pipeline->chunk.mem_end  - pipeline->chunk.mem_begin;

    // skip pathological cases
    if (n_mems == 0u)
        return;

    //
    // Here we are going to run multiple passes of the same kernel, as we cannot fit
    // all chains in local memory at once...
    //

    // prepare some ping-pong queues for tracking active reads that need more passes
    nvbio::vector<device_tag,uint32> active_reads( n_reads );
    nvbio::vector<device_tag,uint8>  active_flags( n_reads );
    nvbio::vector<device_tag,uint32> out_reads( n_reads );
    nvbio::vector<device_tag,uint8>  temp_storage;

    // initialize the active reads queue
    thrust::copy(
        thrust::make_counting_iterator<uint32>(0u) + pipeline->chunk.read_begin,
        thrust::make_counting_iterator<uint32>(0u) + pipeline->chunk.read_end,
        active_reads.begin() );

    uint32 n_active = n_reads;

    for (uint32 pass_number = 0u; n_active; ++pass_number)
    {
        const uint32 block_dim = 128;
        const uint32 n_blocks  = util::divide_ri( n_active, block_dim );

        // assign a chain id to each mem
        build_chains_kernel<<<n_blocks, block_dim>>>(
            pipeline->chunk,
            pass_number,
            n_active,
            nvbio::plain_view( active_reads ),
            nvbio::plain_view( active_flags ),
            command_line_options.w,
            command_line_options.max_chain_gap,
            n_mems,
            nvbio::plain_view( chn->mems ),
            nvbio::plain_view( chn->mems_index ),
            nvbio::plain_view( chn->mems_chain ) );

        optional_device_synchronize();
        cuda::check_error("build-chains kernel");

        // shrink the set of active reads
        n_active = copy_flagged(
            n_active,                                   // the number of input elements
            active_reads.begin(),                       // the input sequence of elements to copy
            active_flags.begin(),                       // the input sequence of copy flags
            out_reads.begin(),                          // the output sequence of copied elements
            temp_storage );                             // some temporary storage

        active_reads.swap( out_reads );
    }

    // sort mems by chain id
    // NOTE: it's important here to use a stable-sort, so as to guarantee preserving
    // the ordering by left-coordinate of the MEMs
    thrust::sort_by_key(                                // TODO: this is slow, switch to nvbio::cuda::SortEnactor
        chn->mems_chain.begin(),
        chn->mems_chain.begin() + n_mems,
        chn->mems_index.begin() );

    optional_device_synchronize();
    nvbio::cuda::check_error("build-chains kernel");
}
