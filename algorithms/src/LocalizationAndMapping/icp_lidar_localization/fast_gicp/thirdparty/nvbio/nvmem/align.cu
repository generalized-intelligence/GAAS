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

#include "align.h"
#include "mem-search.h"
#include "options.h"
#include "pipeline.h"
#include "util.h"

#include <nvbio/basic/numbers.h>
#include <nvbio/basic/algorithms.h>
#include <nvbio/basic/timer.h>
#include <nvbio/basic/transform_iterator.h>
#include <nvbio/basic/vector_view.h>
#include <nvbio/basic/primitives.h>
#include <nvbio/alignment/alignment.h>
#include <nvbio/alignment/batched.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/iterator/transform_iterator.h>

using namespace nvbio;

// initialize the alignment pipeline
//
void align_init(struct pipeline_state *pipeline, const io::SequenceDataDevice *batch)
{
    struct chains_state<device_tag>    *chn = &pipeline->chn;
    struct alignment_state<device_tag> *aln = &pipeline->aln;

    const uint32 n_reads  = pipeline->chunk.read_end - pipeline->chunk.read_begin;
    const uint32 n_chains = chn->n_chains;

    // initially, target the device
    pipeline->system = DEVICE;

    // reserve enough storage
    if (aln->stencil.size() < n_reads)
    {
        aln->begin_chains.clear(); aln->begin_chains.resize( n_reads );
        aln->end_chains.clear();   aln->end_chains.resize( n_reads );
        aln->stencil.clear();      aln->stencil.resize( n_reads );
        aln->temp_queue.clear();   aln->temp_queue.resize( n_reads );
        aln->query_spans.clear();  aln->query_spans.resize( n_reads );
        aln->ref_spans.clear();    aln->ref_spans.resize( n_reads );
        aln->sinks.clear();        aln->sinks.resize( n_reads );
    }

    // find the first chain for each read
    thrust::lower_bound(
        chn->chain_reads.begin(),
        chn->chain_reads.begin() + n_chains,
        thrust::make_counting_iterator<uint32>( pipeline->chunk.read_begin ),
        thrust::make_counting_iterator<uint32>( pipeline->chunk.read_end ),
        aln->begin_chains.begin() );

    // find the ending chain for each read
    thrust::upper_bound(
        chn->chain_reads.begin(),
        chn->chain_reads.begin() + n_chains,
        thrust::make_counting_iterator<uint32>( pipeline->chunk.read_begin ),
        thrust::make_counting_iterator<uint32>( pipeline->chunk.read_end ),
        aln->end_chains.begin() );

    aln->n_active = n_reads;
}

#define MEM_SHORT_EXT 50
#define MEM_SHORT_LEN 200

// a functor to compute the size of a span
//
struct span_size
{
    typedef uint2   argument_type;
    typedef uint32  result_type;

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 operator() (const uint2 span) const { return span.y - span.x; }
};

// a functor to extract the reference span from a chain
//
struct span_functor
{
    typedef io::SequenceDataAccess<DNA_N>   reads_access_type;
    typedef io::SequenceDataAccess<DNA>     reference_access_type;

    NVBIO_HOST_DEVICE
    span_functor(
        const runtime_options       _options,
        const reads_access_type     _reads,
        const reference_access_type _reference,
        const chains_view           _chains,
        const uint32*               _active_chains,
              uint2*                _query_spans,
              uint2*                _ref_spans,
              uint8*                _flags) :
        options         ( _options ),
        reads           ( _reads ),
        reference       ( _reference ),
        chains          ( _chains ),
        active_chains   ( _active_chains ),
        query_spans     ( _query_spans ),
        ref_spans       ( _ref_spans ),
        flags           ( _flags ) {}

    // the functor operator
    NVBIO_HOST_DEVICE
    void operator() (const uint32 idx) const
    {
        const uint32          chain_idx = active_chains[idx];
        const chain_reference chain     = chains[chain_idx];

        const uint32 len = chain.size();

        uint2 qspan = make_uint2( uint32(-1), 0u );
        uint2 rspan = make_uint2( uint32(-1), 0u );

        // loop through all seeds in this chain
        for (uint32 i = 0; i < len; ++i)
        {
            // fetch the i-th seed
            const chains_view::mem_type seed = chain[i];

            qspan.x = nvbio::min( qspan.x, seed.span().x );
            qspan.y = nvbio::max( qspan.y, seed.span().y );

            rspan.x = nvbio::min( rspan.x, seed.index_pos() );
            rspan.y = nvbio::max( rspan.y, seed.index_pos() + seed.span().y - seed.span().x );
        }

        const uint32 read_id    = chain.read();
        const uint2  read_range = reads.get_range( read_id );
        const uint32 read_len   = read_range.y - read_range.x;

        qspan.x = qspan.x > MEM_SHORT_EXT            ? qspan.x - MEM_SHORT_EXT : 0u;
        qspan.y = qspan.y + MEM_SHORT_EXT < read_len ? qspan.y + MEM_SHORT_EXT : read_len;

        rspan.x = rspan.x > MEM_SHORT_EXT                   ? rspan.x - MEM_SHORT_EXT : 0u;
        rspan.y = rspan.y + MEM_SHORT_EXT < reference.bps() ? rspan.y + MEM_SHORT_EXT : reference.bps();

        const uint32 qdelta = qspan.y - qspan.x;
        const uint32 rdelta = rspan.y - rspan.x;

        if ((qspan.x <= 10 || qspan.y >= read_len - 10) ||            // because ksw_align() does not support end-to-end alignment
            (rdelta > qdelta + MEM_SHORT_EXT || qdelta > rdelta + MEM_SHORT_EXT) ||
            (qdelta >= options.w * 4 || rdelta >= options.w * 4))
        {
            flags[idx] = 0; // because ksw_align() does not support end-to-end alignment
            return;
        }

        // save the resulting spans
        query_spans[idx] = make_uint2( qspan.x + read_range.x, qspan.y + read_range.x );
          ref_spans[idx] = rspan;

        // flag to perform short alignment
        flags[idx] = 1;
    }

    const runtime_options       options;
    const reads_access_type     reads;
    const reference_access_type reference;
    const chains_view           chains;
    const uint32*               active_chains;
          uint2*                query_spans;
          uint2*                ref_spans;
          uint8*                flags;
};


// perform banded alignment
//
template <typename system_tag>
uint32 align_short(
    chains_state<system_tag>            *chn,
    alignment_state<system_tag>         *aln,
    const io::SequenceData              *reference,
    const io::SequenceData              *reads)
{
    typedef io::SequenceDataAccess<DNA_N>               read_access_type;
    typedef io::SequenceDataAccess<DNA>                 reference_access_type;

    // prepare POD access pointers to the reads and reference
    const read_access_type      reads_access( *reads );
    const reference_access_type reference_access( *reference );

    //
    // During alignment, we essentially keep a queue of "active" reads, corresponding
    // to those reads for which there's more chains to process; at every step, we select
    // one new chain from each read as an alignment candidate, removing it from the set.
    // This is done keeping a set of (begin,end) pointers per read and advancing the
    // begin field - when a range becomes empty, it's removed
    //
    uint32 n_active = aln->n_active;

    // build a stencil of the active reads, stencil[i] = (begin_chains[i] != end_chains[i])
    transform<system_tag>(
        n_active,
        aln->begin_chains.begin(),
        aln->end_chains.begin(),
        aln->stencil.begin(),
        nvbio::not_equal_functor<uint32>() );

    nvbio::vector<system_tag,uint8> temp_storage;

    // filter away reads that are done processing because there's no more chains
    copy_flagged(
        n_active,
        aln->begin_chains.begin(),
        aln->stencil.begin(),
        aln->temp_queue.begin(),
        temp_storage );

    aln->begin_chains.swap( aln->temp_queue );

    n_active = copy_flagged(
        n_active,
        aln->end_chains.begin(),
        aln->stencil.begin(),
        aln->temp_queue.begin(),
        temp_storage );

    aln->end_chains.swap( aln->temp_queue );

    // reset the number of active reads
    aln->n_active = n_active;

    // check whether there's no more work to do
    if (n_active == 0)
        return 0u;

    // now build a view of the chains
    const chains_view chains( *chn );

    typedef typename alignment_state<system_tag>::sink_type  sink_type;

    const nvbio::vector<system_tag,uint32>&     cur_chains  = aln->begin_chains;
          nvbio::vector<system_tag,uint2>&      query_spans = aln->query_spans;
          nvbio::vector<system_tag,uint2>&      ref_spans   = aln->ref_spans;
          nvbio::vector<system_tag,uint8>&      stencil     = aln->stencil;
          nvbio::vector<system_tag,uint32>&     list        = aln->temp_queue;
          nvbio::vector<system_tag,sink_type>&  sinks       = aln->sinks;

    // compute the chain query-spans
    for_each<system_tag>(
        n_active,
        thrust::make_counting_iterator<uint32>(0u),
        span_functor(
            command_line_options,
            reads_access,
            reference_access,
            chains,
            raw_pointer( cur_chains ),
            raw_pointer( query_spans ),
            raw_pointer( ref_spans ),
            raw_pointer( stencil ) ) );

    // copy the list of indices to the short alignment problems
    const uint32 n_alns = copy_flagged(
        n_active,
        thrust::make_counting_iterator<uint32>(0u),
        stencil.begin(),
        list.begin(),
        temp_storage );

    if (n_alns)
    {
        //
        // perform a Gotoh batched alignment between two string-sets:
        // the string-sets here are sparse subsets of the symbol streams holding
        // the reads and the reference data
        //

        typedef read_access_type::sequence_stream_type                      read_stream_type;
        typedef reference_access_type::sequence_stream_type                 reference_stream_type;
        typedef thrust::permutation_iterator<const uint2*, const uint32*>   infix_iterator;

        const infix_iterator reads_infixes      = thrust::make_permutation_iterator( raw_pointer( query_spans ), raw_pointer( list ) );
        const infix_iterator reference_infixes  = thrust::make_permutation_iterator( raw_pointer( ref_spans ),   raw_pointer( list ) );

        // build the sparse subset of the reads sequence
        const SparseStringSet<read_stream_type,infix_iterator> read_infix_set(
            n_alns,
            reads_access.sequence_stream(),
            reads_infixes );

        // build the sparse subset of the reference sequence
        const SparseStringSet<reference_stream_type,infix_iterator> reference_infix_set(
            n_alns,
            reference_access.sequence_stream(),
            reference_infixes );

        // compute the largest reference span
        const uint32 max_rspan = nvbio::reduce(
            n_alns,
            thrust::make_transform_iterator( reference_infixes, span_size() ),
            thrust::maximum<uint32>(),
            temp_storage );

        const aln::SimpleGotohScheme gotoh( 2, -2, -5, -3 ); // TODO: assign the correct scores here

        // invoke the parallel alignment
        aln::batch_alignment_score(
            aln::make_gotoh_aligner<aln::LOCAL>( gotoh ),
            read_infix_set,
            reference_infix_set,
            sinks.begin(),
            aln::DeviceThreadScheduler(),
            reads_access.max_sequence_len(),
            max_rspan );

        // TODO:
        //  - check which alignments were successful
        //  - perform a reverse alignment to find the source cell of each alignment
    }
    // add one to the processed chains
    nvbio::transform<system_tag>(
        n_active,
        aln->begin_chains.begin(),
        thrust::make_constant_iterator<uint32>( 1u ),
        aln->begin_chains.begin(),
        nvbio::add_functor() );

    return n_active;
}

// perform banded alignment
//
uint32 align(
    struct pipeline_state               *pipeline,
    const nvbio::io::SequenceDataHost   *reads_host,
    const nvbio::io::SequenceDataDevice *reads_device)
{
    if (pipeline->system == DEVICE &&       // if currently on the device,
        pipeline->aln.n_active < 16*1024)   // but too little parallelism...
    {
        // copy the state of the pipeline to the host
        pipeline->system = HOST;
        pipeline->h_chn = pipeline->chn;
        pipeline->h_aln = pipeline->aln;
    }

    if (pipeline->system == HOST)
    {
        return align_short<host_tag>(
            &pipeline->h_chn,
            &pipeline->h_aln,
            (const io::SequenceData*)pipeline->mem.reference_data_host,
            (const io::SequenceData*)reads_host );
    }
    else
    {
        return align_short<device_tag>(
            &pipeline->chn,
            &pipeline->aln,
            (const io::SequenceData*)pipeline->mem.reference_data_device,
            (const io::SequenceData*)reads_device );
    }
}
