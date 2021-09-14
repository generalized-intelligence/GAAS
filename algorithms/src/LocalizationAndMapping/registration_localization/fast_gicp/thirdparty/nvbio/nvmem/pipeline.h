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

#include <nvbio/io/output/output_file.h>
#include <nvbio/io/sequence/sequence.h>
#include <nvbio/io/fmindex/fmindex.h>
#include <nvbio/alignment/sink.h>
#include <nvbio/fmindex/mem.h>

#include "mem-search.h"

using namespace nvbio;

/// pipeline stats
///
struct pipeline_stats
{
    pipeline_stats() :
        time        ( 0.0f ),
        io_time     ( 0.0f ),
        search_time ( 0.0f ),
        locate_time ( 0.0f ),
        chain_time  ( 0.0f ),
        n_reads     ( 0 ),
        n_mems      ( 0 ),
        n_chains    ( 0 )
    {}

    float time;
    float io_time;
    float search_time;
    float locate_time;
    float chain_time;

    uint64 n_reads;
    uint64 n_mems;
    uint64 n_chains;
};

/// the MEM-searching pipeline state
///
struct mem_state
{
    typedef nvbio::io::SequenceDataAccess<DNA>::sequence_stream_type    genome_type;
    typedef nvbio::io::FMIndexDataDevice::fm_index_type                 fm_index_type;
    typedef nvbio::MEMFilterDevice<fm_index_type>                       mem_filter_type;
    typedef mem_filter_type::mem_type                                   mem_type;

    nvbio::io::SequenceData         *reference_data_host;
    nvbio::io::SequenceDataDevice   *reference_data_device;
    nvbio::io::FMIndexData          *fmindex_data_host;
    nvbio::io::FMIndexDataDevice    *fmindex_data_device;

    fm_index_type                    f_index;           ///< the forward FM-index object
    fm_index_type                    r_index;           ///< the reverse FM-index object

    mem_filter_type                  mem_filter;        ///< our MEM filter object, used to rank and locate MEMs and keep track of statistics
};

/// the state of the MEM chains relative to a set of reads
///
template <typename system_tag>
struct chains_state
{
    typedef mem_state::mem_type                     mem_type;
    typedef nvbio::vector<system_tag, mem_type>     mem_vector_type;

    template <typename other_tag>
    chains_state& operator=(const chains_state<other_tag>& other);

    mem_vector_type                  mems;              ///< the result vector for mem_search
    uint32                           n_mems;            ///< the number of mems

    nvbio::vector<system_tag,uint32> mems_index;        ///< a sorting index into the mems (initially by reference location, then by chain id)
    nvbio::vector<system_tag,uint64> mems_chain;        ///< the chain IDs of each mem

    // the list of chains
    nvbio::vector<system_tag,uint32> chain_offsets;     ///< the first seed of each chain
    nvbio::vector<system_tag,uint32> chain_lengths;     ///< the number of seeds in each chain
    nvbio::vector<system_tag,uint32> chain_reads;       ///< the read (strand) id of each chain
    uint32                           n_chains;          ///< the number of chains
};

/// a small object acting as a "reference" for a chain
///
struct chain_reference;

/// a POD structure to view chains as if they were stored in AOS format;
/// unlike chains_state, this class can be passed as a kernel parameter and its members
/// can be accessed from the device.
///
struct chains_view
{
    typedef mem_state::mem_type                                     mem_type;
    typedef const uint32*                                           index_vector_type;
    typedef const mem_type*                                         mem_vector_type;

    /// constructor
    ///
    template <typename system_tag>
    chains_view(const chains_state<system_tag>& state) :
        mems( plain_view( state.mems ) ),
        mems_index( plain_view( state.mems_index ) ),
        chain_offsets( plain_view( state.chain_offsets ) ),
        chain_lengths( plain_view( state.chain_lengths ) ),
        chain_reads( plain_view( state.chain_reads ) ),
        n_chains( state.n_chains )
    {}

    /// return a "reference" to the i-th chain
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    chain_reference operator[] (const uint32 i) const;

    mem_vector_type   mems;                         ///< the result vector for mem_search
    index_vector_type mems_index;                   ///< a sorting index into the mems (initially by reference location, then by chain id)

    index_vector_type chain_offsets;                ///< the first seed of each chain
    index_vector_type chain_lengths;                ///< the number of seeds in each chain
    index_vector_type chain_reads;                  ///< the read (strand) id of each chain
    uint32            n_chains;                     ///< the number of chains
};

/// a small object acting as a "reference" for a chain, allowing to view it as if was a single object
///
struct chain_reference
{
    typedef mem_state::mem_type mem_type;

    /// constructor
    ///
    NVBIO_HOST_DEVICE
    chain_reference(const chains_view& _chains, const uint32 _i) : chains(_chains), idx(_i) {}

    /// return the read this chain belongs to
    ///
    NVBIO_HOST_DEVICE
    uint32 read() const { return chains.chain_reads[ idx ]; }

    /// return the number of seeds in this chain
    ///
    NVBIO_HOST_DEVICE
    uint32 size() const { return chains.chain_lengths[ idx ]; }

    /// return the i-th seed in this chain
    ///
    NVBIO_HOST_DEVICE
    mem_type operator[] (const uint32 i) const
    {
        // grab the offset to the first seed of this chain
        const uint32 offset = chains.chain_offsets[ idx ];

        // return the requested seed, remembering they are sorted by chain through an index
        return chains.mems[ chains.mems_index[ offset + i ] ];
    }

    const chains_view& chains;
    const uint32       idx;
};

// return a "reference" to the i-th chain
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
chain_reference chains_view::operator[] (const uint32 i) const
{
    return chain_reference( *this, i );
}

/// a contiguous subset of reads from a batch associated with all their MEMs
///
struct read_chunk
{
    read_chunk() :
        read_begin(0),
        read_end(0),
        mem_begin(0),
        mem_end(0) {}

    uint32  read_begin;         ///< ID of the first read in this chunk
    uint32  read_end;           ///< ID of the ending read in this chunk

    uint32  mem_begin;          ///< index of the first hit for the first read
    uint32  mem_end;            ///< index of the last hit for the last read
};

/// the alignment sub-pipeline state
///
/// during alignment, we essentially keep a queue of "active" reads, corresponding
/// to those reads for which there's more chains to process; at every step, we select
/// one new chain from each read as an alignment candidate, removing it from the set.
/// This is done keeping a set of (begin,end) pointers per read and advancing the
/// begin field - when a range becomes empty, it's removed
///
template <typename system_tag>
struct alignment_state
{
    typedef aln::Best2Sink<int16> sink_type;

    template <typename other_tag>
    alignment_state& operator=(const alignment_state<other_tag>& other);

    uint32                               n_active;              ///< the number of active reads in the alignment queue
    nvbio::vector<system_tag,uint32>     begin_chains;          ///< the first chain for each read in the processing queue
    nvbio::vector<system_tag,uint32>     end_chains;            ///< the ending chain for each read in the processing queue
    nvbio::vector<system_tag,uint2>      query_spans;           ///< the query chain spans
    nvbio::vector<system_tag,uint2>      ref_spans;             ///< the reference chain spans
    nvbio::vector<system_tag,uint32>     temp_queue;            ///< a temporary queue
    nvbio::vector<system_tag,uint8>      stencil;                   ///< a temporary stencil vector
    nvbio::vector<system_tag,sink_type>  sinks;  ///< a temporary stencil vector
};

/// a flag to identify the system in use
///
enum SystemFlag { DEVICE = 0, HOST = 1 };

/// the state of the pipeline
///
struct pipeline_state
{
    SystemFlag                  system;                         ///< specify whether we are using the device or the host
    nvbio::io::OutputFile*      output;                         ///< the alignment output
    mem_state                   mem;                            ///< the mem state
    chains_state<device_tag>    chn;                            ///< the device chains state
    alignment_state<device_tag> aln;                            ///< the device alignment state
    chains_state<host_tag>      h_chn;                          ///< the host chains state
    alignment_state<host_tag>   h_aln;                          ///< the host alignment state
    read_chunk                  chunk;                          ///< the current read chunk
    pipeline_stats              stats;                          ///< the pipeline stats
};


template <typename system_tag>
template <typename other_tag>
chains_state<system_tag>& chains_state<system_tag>::operator=(const chains_state<other_tag>& other)
{
    n_mems   = other.n_mems;
    n_chains = other.n_chains;

    mems.resize( n_mems );
    mems_index.resize( n_mems );

    chain_offsets.resize( n_chains );
    chain_lengths.resize( n_chains );
    chain_reads.resize( n_chains );

    thrust::copy( other.mems.begin(),       other.mems.begin()       + n_mems,  mems.begin() );
    thrust::copy( other.mems_index.begin(), other.mems_index.begin() + n_mems,  mems_index.begin() );

    thrust::copy( other.chain_offsets.begin(),  other.chain_offsets.begin() + n_chains, chain_offsets.begin() );
    thrust::copy( other.chain_lengths.begin(),  other.chain_lengths.begin() + n_chains, chain_lengths.begin() );
    thrust::copy( other.chain_reads.begin(),    other.chain_reads.begin()   + n_chains, chain_reads.begin() );
    return *this;
}

template <typename system_tag>
template <typename other_tag>
alignment_state<system_tag>& alignment_state<system_tag>::operator=(const alignment_state<other_tag>& other)
{
    n_active = other.n_active;

    begin_chains.resize( n_active );
    end_chains.resize( n_active );
    query_spans.resize( n_active );
    ref_spans.resize( n_active );
    temp_queue.resize( n_active );
    stencil.resize( n_active );

    thrust::copy( other.begin_chains.begin(),   other.begin_chains.begin() + n_active,  begin_chains.begin() );
    thrust::copy( other.end_chains.begin(),     other.end_chains.begin()   + n_active,  end_chains.begin() );
    thrust::copy( other.query_spans.begin(),    other.query_spans.begin()  + n_active,  query_spans.begin() );
    thrust::copy( other.ref_spans.begin(),      other.ref_spans.begin()    + n_active,  ref_spans.begin() );
    return *this;
}
