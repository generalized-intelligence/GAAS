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

///
///\file scoring_queues.h
///

#pragma once

#include <nvBowtie/bowtie2/cuda/defs.h>
#include <nvbio/basic/cuda/pingpong_queues.h>
#include <nvbio/basic/cuda/arch.h>
#include <nvbio/basic/thrust_view.h>
#include <algorithm>

namespace nvbio {
namespace bowtie2 {
namespace cuda {

///@addtogroup nvBowtie
///@{

/// \defgroup ScoringQueuesModule Scoring Queues
///
/// This module contains a set of classes used to represent the scoring queues employed throughout
/// all of nvBowtie in best mapping mode.
/// Conceptually, the queues (ScoringQueues) represent a set of "active" reads, and a nested sequence of variable length
/// seed hit lists bound to the active reads.
///
/// The hits (HitQueues) in turn are represented by a set of attributes that are computed during different pipeline
/// stages: the hit location, its RC flags, its alignment score, its alignment sink, and so on.
///
/// In order to achieve maximum access performance, all of these attributes are stored in SOA layout:
/// the classes in this module serve the purpose of providing a structured AOS-like view over the raw data.
///

///@addtogroup ScoringQueuesModule
///@{

// forward declaration
template <typename HitQueuesType> struct HitReference;
template <typename ScoringQueuesType> struct ReadHitReference;
template <typename ScoringQueuesType> struct ReadHitBinder;

struct HitQueuesDeviceView;
struct ScoringQueuesDeviceView;
struct ReadHitsIndexDeviceView;

///
/// This data structure represents a collection of hit arrays bound to a set of reads.
/// For each read it stores an array of hits of arbitrary length.
///
/// Internally, the links from a read to its hits are stored inside a contiguous memory arena
/// using the following strided layout:
///
/// \verbatim
/// ******************************************************
/// *  read0   *  read1  *    ...              *  readN  *
/// ******************************************************
///     count  |   count |    ...              |   count 
/// -----------|---------|---------------------|----------
///     hit0   |   hit0  |    ...              |   hit0  
/// -----------|---------|---------------------|----------
///     hit1   |   hit1  |    ...              |   hit1  
/// -----------|---------|---------------------|----------
///      ...   |    ...  |    ...              |    ...  
/// -----------|---------|---------------------|----------
///     hitM   |   hitM  |    ...              |   hitM  
/// ------------------------------------------------------
/// \endverbatim
///
/// In practice, if the arena has size max_size, and the stride is N, one can represent
/// up to M = max_size / N hits per read.
///
struct ReadHitsIndex
{
    typedef thrust::device_vector<uint32>          links_storage_type;
    typedef ReadHitsIndexDeviceView                device_view_type;

    enum Mode {
        SingleHitPerRead        = 0u,
        MultipleHitsPerRead     = 1u,
    };

    /// alloc enough storage for max_size hits
    ///
    uint64 resize(const uint32 max_size, const bool do_alloc);

    /// setup the number of input reads
    ///
    void setup(const uint32 n_hits_per_read, const uint32 in_reads);

    /// return the queue mode
    ///
    Mode mode() const { return m_mode; }

    /// return a view of the data structure
    ///
    ReadHitsIndexDeviceView device_view();

public:
    links_storage_type      m_links;    ///< links arena
    uint32                  m_stride;   ///< access stride
    Mode                    m_mode;     ///< single/multiple hits mode
};

///
/// This data structure represents a collection of hit arrays bound to a set of reads.
/// For each read it stores an array of hits of arbitrary length.
///
struct ReadHitsIndexDeviceView
{
    typedef typename device_view_subtype<ReadHitsIndex::links_storage_type>::type   links_storage_type;

    ///
    /// An auxiliary class to represent the hit array bound to a given read
    ///
    struct HitArray
    {
        /// constructor
        ///
        NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
        HitArray(ReadHitsIndexDeviceView& index, const uint32 read_index) : m_index(index), m_read_index(read_index) {}

        /// set the number of hits
        ///
        NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
        void resize(const uint32 size) const { m_index.set_hit_count( m_read_index, size ); }

        /// return the number of hits
        ///
        NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
        uint32 size() const { return m_index.hit_count( m_read_index ); }

        /// return the i-th hit link
        ///
        NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
        uint32& operator[] (const uint32 slot) { return m_index( m_read_index, slot ); }

        /// return the i-th hit link
        ///
        NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
        uint32 operator[] (const uint32 slot) const { return m_index( m_read_index, slot ); }

    private:
        ReadHitsIndexDeviceView& m_index;
        uint32                   m_read_index;
    };

    // define the reference type
    typedef HitArray    reference;


    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    ReadHitsIndexDeviceView(
        links_storage_type  links  = links_storage_type(),
        const uint32        stride = 0u) : m_links( links ), m_stride( stride ) {}

    /// return the number of hits bound to a read
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 hit_count(const uint32 read_index) const { return m_links[ read_index ]; }

    /// setup the number of hits
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void set_hit_count(const uint32 read_index, const uint32 count) { m_links[ read_index ] = count; }

    /// return the i-th hit link bound to a given read
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32& operator() (const uint32 read_index, const uint32 slot) { return m_links[ read_index + (1u + slot) * m_stride ]; }

    /// return the i-th hit link bound to a given read
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 operator() (const uint32 read_index, const uint32 slot) const { return m_links[ read_index + (1u + slot) * m_stride ]; }

    /// return the hit array bound to a given read
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    HitArray operator[] (const uint32 read_index) { return HitArray( *this, read_index ); }

public:
    links_storage_type  m_links;
    uint32              m_stride;

    friend struct HitArray;
};

///
/// The collection of queues defining the set of hits - each queue holds one element/attribute per hit
///
struct HitQueues
{
    typedef thrust::device_vector<uint32>          index_storage_type;
    typedef thrust::device_vector<packed_seed>     seed_storage_type;
    typedef thrust::device_vector<uint32>          ssa_storage_type;
    typedef thrust::device_vector<uint32>          loc_storage_type;
    typedef thrust::device_vector<int32>           score_storage_type;
    typedef thrust::device_vector<uint32>          sink_storage_type;

    typedef HitQueuesDeviceView                    device_view_type;       ///< view subtype
    typedef HitReference<HitQueues>                reference;              ///< reference subtype

    /// resize all queues
    ///
    uint64 resize(const uint32 size, const bool do_alloc);

    /// return a view of the data structure
    ///
    HitQueuesDeviceView device_view();

    index_storage_type     read_id;            ///< hit -> read mapping
    seed_storage_type      seed;               ///< hit info
    ssa_storage_type       ssa;                ///< hit ssa info
    loc_storage_type       loc;                ///< hit locations
    score_storage_type     score;              ///< hit scores
    sink_storage_type      sink;               ///< hit sinks
    loc_storage_type       opposite_loc;       ///< hit locations, opposite mate
    score_storage_type     opposite_score;     ///< hit scores, opposite mate
    sink_storage_type      opposite_sink;      ///< hit sinks, opposite mate
    score_storage_type     opposite_score2;    ///< hit scores, opposite mate
    sink_storage_type      opposite_sink2;     ///< hit sinks, opposite mate
};
///
/// A view of the HitQueues class
///
struct HitQueuesDeviceView
{
    typedef typename device_view_subtype<HitQueues::index_storage_type>::type            index_storage_type;
    typedef typename device_view_subtype<HitQueues::seed_storage_type>::type             seed_storage_type;
    typedef typename device_view_subtype<HitQueues::ssa_storage_type>::type              ssa_storage_type;
    typedef typename device_view_subtype<HitQueues::loc_storage_type>::type              loc_storage_type;
    typedef typename device_view_subtype<HitQueues::score_storage_type>::type            score_storage_type;
    typedef typename device_view_subtype<HitQueues::sink_storage_type>::type             sink_storage_type;

    typedef HitQueuesDeviceView                    device_view_type;       ///< view subtype
    typedef HitReference<HitQueuesDeviceView>      reference;              ///< reference subtype

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    HitQueuesDeviceView(
        index_storage_type     _read_id         = index_storage_type(),     // hit -> read mapping
        seed_storage_type      _seed            = seed_storage_type(),      // hit info
        ssa_storage_type       _ssa             = ssa_storage_type(),       // hit ssa info
        loc_storage_type       _loc             = loc_storage_type(),       // hit locations
        score_storage_type     _score           = score_storage_type(),     // hit scores
        sink_storage_type      _sink            = sink_storage_type(),      // hit sinks
        loc_storage_type       _opposite_loc    = loc_storage_type(),       // hit locations, opposite mate
        score_storage_type     _opposite_score  = score_storage_type(),     // hit scores, opposite mate
        sink_storage_type      _opposite_sink   = sink_storage_type(),      // hit sinks, opposite mate
        score_storage_type     _opposite_score2 = score_storage_type(),     // hit scores, opposite mate
        sink_storage_type      _opposite_sink2  = sink_storage_type());     // hit sinks, opposite mate

    /// return a reference to a given hit
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    HitReference<HitQueuesDeviceView> operator[] (const uint32 i);

    /// return a reference to a given hit
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    HitReference<HitQueuesDeviceView> operator[] (const uint32 i) const;

    index_storage_type     read_id;            ///< hit -> read mapping
    seed_storage_type      seed;               ///< hit info
    ssa_storage_type       ssa;                ///< hit ssa info
    loc_storage_type       loc;                ///< hit locations
    score_storage_type     score;              ///< hit scores
    sink_storage_type      sink;               ///< hit sinks
    loc_storage_type       opposite_loc;       ///< hit locations, opposite mate
    score_storage_type     opposite_score;     ///< hit scores, opposite mate
    sink_storage_type      opposite_sink;      ///< hit sinks, opposite mate
    score_storage_type     opposite_score2;    ///< hit scores, opposite mate
    sink_storage_type      opposite_sink2;     ///< hit sinks, opposite mate
};

///
/// The collection of queues needed by the scoring pipeline.
/// Particularly:
///
///   - a ping-pong set of input/output active-read queues,
///     storing information about the read-id and some flags representing
///     their pipeline state.
///
///   - a set of hits, and a read->hits index
///     these can essentially be seen as forming a nested sequence of hits,
///     with a variable length set of hits bound to each read.
///
/// The scoring pipeline is essentially a two-stage self-feeding loop:
///
///   In the first stage, a certain amount of hits are selected for scoring
///   for each input active-read.
///   Reads for which no hits are selected become inactive, and reads which
///   stay active are output to the output queue together with their hits.
///
///   In the second stage, the queues are swapped, the output queue becomes
///   the input, and the hits bound ot each active read gets scored.
///
/// This class provides two accessors to represent:
///
///  - "references" to the reads in the input queue (ReadHitsReference), and
///
///  - "binders" to reads in the output queue (ReadHitsBinder),
///    useful for binding output reads in the first stage.
///
struct ScoringQueues
{
    typedef nvbio::cuda::PingPongQueues<packed_read>    active_reads_storage_type;
    typedef HitQueues                                   hits_storage_type;
    typedef ReadHitsIndex                               read_hits_index_type;
    typedef thrust::device_vector<uint32>               pool_type;

    typedef ScoringQueuesDeviceView                     device_view_type;

    /// constructor
    ///
    ScoringQueues() : hits_pool(1) {}

    /// resize
    ///
    uint64 resize(const uint32 n_reads, const uint32 n_hits, const bool do_alloc);

    /// clear output queues
    ///
    void clear_output() { hits_pool[0] = 0u; active_reads.clear_output(); }

    /// return the number of allocated hits
    ///
    uint32 hits_count() { return hits_pool[0]; }

    /// return a view of the data structure
    ///
    ScoringQueuesDeviceView device_view();

    active_reads_storage_type  active_reads;       ///< set of active reads
    hits_storage_type          hits;               ///< nested sequence
    read_hits_index_type       hits_index;         ///< read -> hits mapping
    pool_type                  hits_pool;          ///< pool counter
};
///
/// A view of the ScoringQueues class
///
struct ScoringQueuesDeviceView
{
    typedef typename device_view_subtype<ScoringQueues::active_reads_storage_type>::type    active_reads_storage_type;
    typedef typename device_view_subtype<ScoringQueues::hits_storage_type>::type            hits_storage_type;
    typedef typename device_view_subtype<ScoringQueues::read_hits_index_type>::type         read_hits_index_type;
    typedef typename device_view_subtype<ScoringQueues::pool_type>::type                    pool_type;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    ScoringQueuesDeviceView() {}

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    ScoringQueuesDeviceView(
        active_reads_storage_type  _active_reads,  // set of active reads
        hits_storage_type          _hits,          // nested sequence of read hits
        read_hits_index_type       _hits_index,    // read -> hits mapping
        pool_type                  _hits_pool);    // pool counter

    /// return the number of active reads in the input queue
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 active_read_count() const { return active_reads.in_size; }

    /// return the read info for a given entry of the input queue
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    packed_read active_read(const uint32 read_index) const { return active_reads.in_queue[read_index]; }

    active_reads_storage_type  active_reads;       ///< set of active reads
    hits_storage_type          hits;               ///< nested sequence of read hits
    read_hits_index_type       hits_index;         ///< read -> hits mapping
    pool_type                  hits_pool;          ///< pool counter
};

///
/// A reference to a hit
///
/// \tparam HitQueuesType can be either HitQueues or HitQueuesView.
///
template <typename HitQueuesType>
struct HitReference
{
    typedef typename reference_subtype<typename HitQueuesType::index_storage_type>::type           index_type;
    typedef typename reference_subtype<typename HitQueuesType::seed_storage_type>::type            seed_type;
    typedef typename reference_subtype<typename HitQueuesType::ssa_storage_type>::type             ssa_type;
    typedef typename reference_subtype<typename HitQueuesType::loc_storage_type>::type             loc_type;
    typedef typename reference_subtype<typename HitQueuesType::score_storage_type>::type           score_type;
    typedef typename reference_subtype<typename HitQueuesType::sink_storage_type>::type            sink_type;

    /// constructor
    ///
    /// \param hits           hits container
    /// \param hit_index      index of this hit
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    HitReference(HitQueuesType& hits, const uint32 hit_index);

    index_type  read_id;           ///< parent read index
    seed_type   seed;              ///< seed reference
    ssa_type    ssa;               ///< ssa reference
    loc_type    loc;               ///< loc reference
    score_type  score;             ///< score reference
    sink_type   sink;              ///< sink reference
    loc_type    opposite_loc;      ///< opposite mate's loc reference
    score_type  opposite_score;    ///< opposite mate's score reference
    sink_type   opposite_sink;     ///< opposite mate's sink reference
    score_type  opposite_score2;   ///< opposite mate's score reference
    sink_type   opposite_sink2;    ///< opposite mate's sink reference
};

///
/// A reference to the collection of hits bound to a given read (in the input queue)
///
template <typename ScoringQueuesType>
struct ReadHitsReference
{
    typedef typename ScoringQueuesType::read_hits_index_type  read_hits_index_type;
    typedef typename ScoringQueuesType::hits_storage_type     hits_storage_type;
    typedef HitReference<hits_storage_type>                   reference;

    /// constructor
    ///
    /// \param queues             scoring queues
    /// \param read_index         index of this read
    /// \param selector           input/output queue selector
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    ReadHitsReference(ScoringQueuesType& queues, const uint32 read_index = uint32(-1));

    /// bind this object to a given index
    ///
    /// \param read_index         output index of this read
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void bind(const uint32 read_index);

    /// size of the hits vector
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 size() const;

    /// return the i-th element
    /// NOTE: this method is only valid after the i-th hit has been bound.
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    HitReference<hits_storage_type> operator[] (const uint32 i) const;

    /// access the packed_read info in the selected queue
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    packed_read read_info() const;

    /// return the slot where the i-th element is stored
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 slot(const uint32 i) const;

private:
    ScoringQueuesType& m_queues;       ///< view of the hits container
    uint32             m_read_index;   ///< read index
};


///
/// A class used to bind a read and its hits to the output queue
///
template <typename ScoringQueuesType>
struct ReadHitsBinder
{
    typedef typename ScoringQueuesType::read_hits_index_type  read_hits_index_type;
    typedef typename ScoringQueuesType::hits_storage_type     hits_storage_type;
    typedef HitReference<hits_storage_type>                   reference;

    /// constructor
    ///
    /// \param queues             scoring queues
    /// \param read_index         index of this read
    /// \param selector           input/output queue selector
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    ReadHitsBinder(ScoringQueuesType& queues, const uint32 read_index = uint32(-1));

    /// bind this object to a given index
    ///
    /// \param read_index         output index of this read
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void bind(const uint32 read_index);

    /// resize the hits vector
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void resize(const uint32 size);

    /// size of the hits vector
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 size() const;

    /// return the i-th element
    /// NOTE: this method is only valid after the i-th hit has been bound.
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    HitReference<hits_storage_type> operator[] (const uint32 i) const;

    /// access the packed_read info in the selected queue
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    packed_read read_info() const;

    /// set the read info
    ///
    /// \param read_index         output index of this read
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void set_read_info(const packed_read info);

    /// bind the i-th hit to a given location
    ///
    /// \param i        index of the hit to bind relative to this read
    /// \param slot     address of the bound hit in the HitQueues
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void bind_hit(const uint32 i, const uint32 slot);

private:
    ScoringQueuesType& m_queues;       ///< view of the hits container
    uint32             m_read_index;   ///< read index
};

///@}  // group ScoringQueuesModule
///@}  // group nvBowtie

} // namespace cuda
} // namespace bowtie2

// return a view of the queues
//
inline
bowtie2::cuda::ScoringQueuesDeviceView device_view(bowtie2::cuda::ScoringQueues& queues) { return queues.device_view(); }

// return a view of the queues
//
inline
bowtie2::cuda::HitQueuesDeviceView device_view(bowtie2::cuda::HitQueues& queues) { return queues.device_view(); }

// return a view of the index
//
inline
bowtie2::cuda::ReadHitsIndexDeviceView device_view(bowtie2::cuda::ReadHitsIndex& index) { return index.device_view(); }

} // namespace nvbio

#include <nvBowtie/bowtie2/cuda/scoring_queues_inl.h>
