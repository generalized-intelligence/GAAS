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

namespace nvbio {
namespace bowtie2 {
namespace cuda {

// alloc enough storage for max_size hits
//
inline
uint64 ReadHitsIndex::resize(const uint32 max_size, const bool do_alloc)
{
    const uint32 n = 2u*max_size;
    if (do_alloc) m_links.resize( n );
    return n * sizeof(uint32);
}

// setup the number of input reads
//
inline
void ReadHitsIndex::setup(const uint32 n_hits_per_read, const uint32 in_reads)
{
    m_mode = n_hits_per_read > 1 ?
        MultipleHitsPerRead :
        SingleHitPerRead;

    m_stride = in_reads;

    if (n_hits_per_read == 1)
    {
        // fill the links structure
        thrust::fill(
            m_links.begin(),
            m_links.begin() + in_reads,
            uint32(1u) );

        // fill the links structure
        thrust::copy(
            thrust::make_counting_iterator(0u),
            thrust::make_counting_iterator(in_reads),
            m_links.begin() + in_reads );
    }
}

// return a view of the data structure
//
inline
ReadHitsIndexDeviceView ReadHitsIndex::device_view()
{
    return ReadHitsIndexDeviceView(
        nvbio::device_view( m_links ),
        m_stride );
}

// setup all queues
//
inline
uint64 HitQueues::resize(const uint32 size, const bool do_alloc)
{
    uint64 bytes = 0;
    if (do_alloc) read_id.resize( size );         bytes += size * sizeof(uint32);
    if (do_alloc) seed.resize( size );            bytes += size * sizeof(packed_seed);
    if (do_alloc) ssa.resize( size );             bytes += size * sizeof(uint32);
    if (do_alloc) loc.resize( size );             bytes += size * sizeof(uint32);
    if (do_alloc) score.resize( size );           bytes += size * sizeof(int32);
    if (do_alloc) sink.resize( size );            bytes += size * sizeof(uint32);
    if (do_alloc) opposite_loc.resize( size );    bytes += size * sizeof(uint32);
    if (do_alloc) opposite_score.resize( size );  bytes += size * sizeof(int32);
    if (do_alloc) opposite_sink.resize( size );   bytes += size * sizeof(uint32);
    if (do_alloc) opposite_score2.resize( size ); bytes += size * sizeof(int32);
    if (do_alloc) opposite_sink2.resize( size );  bytes += size * sizeof(uint32);
    return bytes;
}

// return a view of the data structure
//
inline
HitQueuesDeviceView HitQueues::device_view()
{
    return HitQueuesDeviceView(
        nvbio::device_view( read_id ),
        nvbio::device_view( seed ),
        nvbio::device_view( ssa ),
        nvbio::device_view( loc ),
        nvbio::device_view( score ),
        nvbio::device_view( sink ),
        nvbio::device_view( opposite_loc ),
        nvbio::device_view( opposite_score ),
        nvbio::device_view( opposite_sink ),
        nvbio::device_view( opposite_score2 ),
        nvbio::device_view( opposite_sink2 ) );
}

// constructor
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
HitQueuesDeviceView::HitQueuesDeviceView(
    index_storage_type     _read_id,            // hit -> read mapping
    seed_storage_type      _seed,               // hit info
    ssa_storage_type       _ssa,                // hit ssa info
    loc_storage_type       _loc,                // hit locations
    score_storage_type     _score,              // hit scores
    sink_storage_type      _sink,               // hit sinks
    loc_storage_type       _opposite_loc,       // hit locations, opposite mate
    score_storage_type     _opposite_score,     // hit scores, opposite mate
    sink_storage_type      _opposite_sink,      // hit sinks, opposite mate
    score_storage_type     _opposite_score2,    // hit scores, opposite mate
    sink_storage_type      _opposite_sink2) :   // hit sinks, opposite mate
    read_id( _read_id ),
    seed( _seed ),
    ssa( _ssa ),
    loc( _loc ),
    score( _score ),
    sink( _sink ),
    opposite_loc( _opposite_loc ),
    opposite_score( _opposite_score ),
    opposite_sink( _opposite_sink ),
    opposite_score2( _opposite_score2 ),
    opposite_sink2( _opposite_sink2 )
{}

// resize
//
inline
uint64 ScoringQueues::resize(const uint32 n_reads, const uint32 n_hits, const bool do_alloc)
{
    uint64 bytes = 0;
    bytes += active_reads.resize_arena( n_reads, do_alloc );
    bytes += hits.resize( n_hits, do_alloc );
    bytes += hits_index.resize( n_hits, do_alloc );
    return bytes;
}

// return a reference to a given hit
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
HitReference<HitQueuesDeviceView> HitQueuesDeviceView::operator[] (const uint32 i)
{
    return HitReference<HitQueuesDeviceView>( *this, i );
}

// return a reference to a given hit
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
HitReference<HitQueuesDeviceView> HitQueuesDeviceView::operator[] (const uint32 i) const
{
    return HitReference<HitQueuesDeviceView>( *const_cast<HitQueuesDeviceView*>( this ), i );
}

// constructor
//
// \param hits           hits container
// \param hit_index      index of this hit
#pragma hd_warning_disable 
template <typename HitQueuesType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
HitReference<HitQueuesType>::HitReference(HitQueuesType& hits, const uint32 hit_index) :
    read_id         ( hits.read_id[ hit_index ] ),
    seed            ( hits.seed[ hit_index ] ),
    ssa             ( hits.ssa[ hit_index ] ),
    loc             ( hits.loc[ hit_index ] ),
    score           ( hits.score[ hit_index ] ),
    sink            ( hits.sink[ hit_index ] ),
    opposite_loc    ( hits.opposite_loc[ hit_index ] ),
    opposite_score  ( hits.opposite_score[ hit_index ] ),
    opposite_sink   ( hits.opposite_sink[ hit_index ] ),
    opposite_score2 ( hits.opposite_score2[ hit_index ] ),
    opposite_sink2  ( hits.opposite_sink2[ hit_index ] )
{}

// return a view of the data structure
//
inline
ScoringQueuesDeviceView ScoringQueues::device_view()
{
    return ScoringQueuesDeviceView(
        nvbio::device_view( active_reads ),
        nvbio::device_view( hits ),
        nvbio::device_view( hits_index ),
        nvbio::device_view( hits_pool ) );
}

// constructor
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
ScoringQueuesDeviceView::ScoringQueuesDeviceView(
    active_reads_storage_type  _active_reads,       // set of active reads
    hits_storage_type          _hits,               // nested sequence of read hits
    read_hits_index_type       _hits_index,         // read -> hits mapping
    pool_type                  _hits_pool) :        // pool counter
    active_reads ( _active_reads ),
    hits         ( _hits ),
    hits_index   ( _hits_index ),
    hits_pool    ( _hits_pool )
{}

// constructor
//
// \param index              read -> hits index
// \param hits               hits container
// \param read_index         index of this read
template <typename ScoringQueuesType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
ReadHitsReference<ScoringQueuesType>::ReadHitsReference(ScoringQueuesType& queues, const uint32 read_index) :
    m_queues( queues ),
    m_read_index( read_index )
{}

// size of the hits vector
//
template <typename ScoringQueuesType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32 ReadHitsReference<ScoringQueuesType>::size() const
{
    return m_queues.hits_index.hit_count( m_read_index );
}

// return the i-th element
//
#pragma hd_warning_disable 
template <typename ScoringQueuesType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
typename ReadHitsReference<ScoringQueuesType>::reference
ReadHitsReference<ScoringQueuesType>::operator[] (const uint32 i) const
{
    const uint32 hit_index = m_queues.hits_index( m_read_index, i );
    return m_queues.hits[ hit_index ];
}

// return the slot where the i-th element is stored
//
#pragma hd_warning_disable 
template <typename ScoringQueuesType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32
ReadHitsReference<ScoringQueuesType>::slot(const uint32 i) const
{
    return m_queues.hits_index( m_read_index, i );
}

// bind this read to its new output location
//
// \param read_index         output index of this read
template <typename ScoringQueuesType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void ReadHitsReference<ScoringQueuesType>::bind(const uint32 read_index)
{
    m_read_index = read_index;
}

// access the packed_read info in the selected queue
//
template <typename ScoringQueuesType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
packed_read ReadHitsReference<ScoringQueuesType>::read_info() const
{
    return m_queues.active_reads.in_queue[ m_read_index ];
}


// constructor
//
// \param index              read -> hits index
// \param hits               hits container
// \param read_index         index of this read
template <typename ScoringQueuesType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
ReadHitsBinder<ScoringQueuesType>::ReadHitsBinder(ScoringQueuesType& queues, const uint32 read_index) :
    m_queues( queues ),
    m_read_index( read_index )
{}

// bind this read to its new output location
//
// \param read_index         output index of this read
template <typename ScoringQueuesType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void ReadHitsBinder<ScoringQueuesType>::bind(const uint32 read_index)
{
    m_read_index = read_index;
}

// resize the hits vector
//
template <typename ScoringQueuesType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void ReadHitsBinder<ScoringQueuesType>::resize(const uint32 size)
{
    m_queues.hits_index.set_hit_count( m_read_index, size );
}

// size of the hits vector
//
template <typename ScoringQueuesType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32 ReadHitsBinder<ScoringQueuesType>::size() const
{
    return m_queues.hits_index.hit_count( m_read_index );
}

// return the i-th element
//
#pragma hd_warning_disable 
template <typename ScoringQueuesType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
typename ReadHitsBinder<ScoringQueuesType>::reference
ReadHitsBinder<ScoringQueuesType>::operator[] (const uint32 i) const
{
    const uint32 hit_index = m_queues.hits_index( m_read_index, i );
    return m_queues.hits[ hit_index ];
}

// access the packed_read info in the selected queue
//
template <typename ScoringQueuesType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
packed_read ReadHitsBinder<ScoringQueuesType>::read_info() const
{
    return m_queues.active_reads.out_queue[ m_read_index ];
}

// set the packed_read info
//
// \param read_index         output index of this read
template <typename ScoringQueuesType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void ReadHitsBinder<ScoringQueuesType>::set_read_info(const packed_read info)
{
    m_queues.active_reads.out_queue[ m_read_index ] = info;
}

// bind the i-th hit to a given location
//
// \param i        index of the hit to bind relative to this read
// \param slot     address of the bound hit in the HitQueues
template <typename ScoringQueuesType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void ReadHitsBinder<ScoringQueuesType>::bind_hit(const uint32 i, const uint32 slot)
{
    m_queues.hits_index( m_read_index, i ) = slot;
}

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
