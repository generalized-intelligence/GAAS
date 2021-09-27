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

namespace nvbio {

namespace qgram {

// return the size of a given range
struct range_size
{
    typedef uint2  argument_type;
    typedef uint64 result_type;

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint64 operator() (const uint2 range) const { return range.y - range.x; }
};

// given a (qgram-pos, text-pos) pair, return the closest regularly-spaced diagonal
template <typename hit_type>
struct closest_diagonal {};

// given a (qgram-pos, text-pos) pair, return the closest regularly-spaced diagonal
template <>
struct closest_diagonal<uint2>
{
    typedef uint2  argument_type;
    typedef uint32 result_type;

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    closest_diagonal(const uint32 _interval) : interval(_interval) {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    result_type operator() (const uint2 range) const
    {
        const uint32 diag = /*qgram_index_string_size + */ range.y - range.x;
        return util::round( diag, interval );
    }

    const uint32 interval;
};

// given a (qgram-id, q-gram-pos, text-pos) pair, return the closest regularly-spaced diagonal
// note:
//  in this case the output type is a (qgram-id,diag) uint2
//
template <>
struct closest_diagonal<uint4>
{
    typedef uint4  argument_type;
    typedef uint2  result_type;

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    closest_diagonal(const uint32 _interval) : interval(_interval) {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    result_type operator() (const uint4 range) const
    {
        const uint32 diag = /*qgram_index.string_length(range.x) + */ range.z - range.y;
        const uint32 rounded_diag = util::round( diag, interval );

        return make_uint2( rounded_diag, range.x );
    }

    const uint32 interval;
};

template <typename qgram_index_type, typename index_iterator, typename coord_type>
struct filter_results {};

template <typename qgram_index_type, typename index_iterator>
struct filter_results< qgram_index_type, index_iterator, uint32 >
{
    typedef uint64  argument_type;
    typedef uint2   result_type;

    // constructor
    filter_results(
        const qgram_index_type  _qgram_index,
        const uint32            _n_queries,
        const uint64*           _slots,
        const uint2*            _ranges,
        const index_iterator    _index) :
    qgram_index ( _qgram_index ),
    n_queries   ( _n_queries ),
    slots       ( _slots ),
    ranges      ( _ranges ),
    index       ( _index ) {}

    // functor operator
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    result_type operator() (const uint64 output_index) const
    {
        // find the text q-gram slot corresponding to this output index
        const uint32 slot = uint32( upper_bound(
            output_index,
            slots,
            n_queries ) - slots );

        // fetch the corresponding text position
        const uint32 text_pos    = index[ slot ];

        // locate the hit q-gram position
        const uint2  range       = ranges[ slot ];
        const uint64 base_slot   = slot ? slots[ slot-1 ] : 0u;
        const uint32 local_index = output_index - base_slot;

        const uint32 qgram_pos = qgram_index.locate( range.x + local_index );

        // and write out the pair (qgram_pos,text_pos)
        return make_uint2( qgram_pos, text_pos );
    }

    const qgram_index_type  qgram_index;
    const uint32            n_queries;
    const uint64*           slots;
    const uint2*            ranges;
    const index_iterator    index;
};

template <typename qgram_index_type, typename index_iterator>
struct filter_results< qgram_index_type, index_iterator, uint2 >
{
    typedef uint64  argument_type;
    typedef uint4   result_type;

    // constructor
    filter_results(
        const qgram_index_type  _qgram_index,
        const uint32            _n_queries,
        const uint64*           _slots,
        const uint2*            _ranges,
        const index_iterator    _index) :
    qgram_index ( _qgram_index ),
    n_queries   ( _n_queries ),
    slots       ( _slots ),
    ranges      ( _ranges ),
    index       ( _index ) {}

    // functor operator
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    result_type operator() (const uint64 output_index) const
    {
        // find the text q-gram slot corresponding to this output index
        const uint32 slot = uint32( upper_bound(
            output_index,
            slots,
            n_queries ) - slots );

        // fetch the corresponding text position
        const uint32 text_pos    = index[ slot ];

        // locate the hit q-gram position
        const uint2  range       = ranges[ slot ];
        const uint32 base_slot   = slot ? slots[ slot-1 ] : 0u;
        const uint32 local_index = output_index - base_slot;

        const uint2 qgram_pos = qgram_index.locate( range.x + local_index );

        // and write out the tuple (index-id,index-pos,text-pos)
        return make_uint4( qgram_pos.x, qgram_pos.y, text_pos, 0u );
    }

    const qgram_index_type  qgram_index;
    const uint32            n_queries;
    const uint64*           slots;
    const uint2*            ranges;
    const index_iterator    index;
};

} // namespace qgram 

// enact the q-gram filter
//
// \param qgram_index      the q-gram index
// \param n_queries        the number of query q-grams
// \param queries          the query q-grams
// \param indices          the query indices
//
template <typename qgram_index_type, typename query_iterator, typename index_iterator>
uint64 QGramFilter<host_tag, qgram_index_type, query_iterator, index_iterator>::rank(
    const qgram_index_type& qgram_index,
    const uint32            n_queries,
    const query_iterator    queries,
    const index_iterator    indices)
{
    typedef typename qgram_index_type::coord_type coord_type;

    // save the query
    m_n_queries   = n_queries;
    m_queries     = queries;
    m_indices     = indices;
    m_qgram_index = nvbio::plain_view( qgram_index );

    // alloc enough storage for the results
    m_ranges.resize( n_queries );
    m_slots.resize( n_queries );

    // search the q-grams in the index, obtaining a set of ranges
    thrust::transform(
        queries,
        queries + n_queries,
        m_ranges.begin(),
        m_qgram_index );

    // scan their size to determine the slots
    thrust::inclusive_scan(
        thrust::make_transform_iterator( m_ranges.begin(), qgram::range_size() ),
        thrust::make_transform_iterator( m_ranges.begin(), qgram::range_size() ) + n_queries,
        m_slots.begin() );

    // determine the total number of occurrences
    m_n_occurrences = m_slots[ n_queries-1 ];
    return m_n_occurrences;
}

// enumerate all hits in a given range
//
template <typename qgram_index_type, typename query_iterator, typename index_iterator>
template <typename hits_iterator>
void QGramFilter<host_tag, qgram_index_type, query_iterator, index_iterator>::locate(
    const uint64            begin,
    const uint64            end,
    hits_iterator           hits)
{
    typedef typename qgram_index_type::coord_type coord_type;

    // and fill it
    thrust::transform(
        thrust::make_counting_iterator<uint64>(0u) + begin,
        thrust::make_counting_iterator<uint64>(0u) + end,
        hits,
        qgram::filter_results<qgram_index_view,index_iterator,coord_type>(
            m_qgram_index,
            m_n_queries,
            nvbio::plain_view( m_slots ),
            nvbio::plain_view( m_ranges ),
            m_indices ) );
}

// simply convert hits to diagonal coordinates
//
// \tparam hits_iterator         a hit_iterator iterator
// \tparam output_iterator       a diagonal_type iterator
//
// \param  n_hits          the number of input hits
// \param  hits            the input hits
// \param  diags           the output diagonals
//
template <typename qgram_index_type, typename query_iterator, typename index_iterator>
template <typename hits_iterator, typename output_iterator>
void QGramFilter<host_tag, qgram_index_type, query_iterator, index_iterator>::diagonals(
    const uint32            n_hits,
    const hits_iterator     hits,
          output_iterator   diags,
    const uint32            interval)
{
    // snap the diagonals to the closest one
    thrust::transform(
        hits,
        hits + n_hits,
        diags,
        qgram::closest_diagonal<hit_type>( interval ) );
}

// merge hits falling within the same diagonal interval; this method will
// replace the vector of hits with a compacted list of hits snapped to the
// closest sample diagonal (i.e. multiple of the given interval), together
// with a counts vector providing the number of hits falling on the same
// spot
//
// \param  interval        the merging interval
// \param  n_hits          the number of input hits
// \param  hits            the input hits
// \param  merged_hits     the output merged hits
// \param  merged_counts   the output merged counts
// \return                 the number of merged hits
//
template <typename qgram_index_type, typename query_iterator, typename index_iterator>
template <typename hits_iterator, typename output_iterator, typename count_iterator>
uint32 QGramFilter<host_tag, qgram_index_type, query_iterator, index_iterator>::merge(
    const uint32            interval,
    const uint32            n_hits,
    const hits_iterator     hits,
          output_iterator   merged_hits,
          count_iterator    merged_counts)
{
    m_diags.resize( n_hits );

    // convert hits to diagonals and snap them to the closest one
    diagonals( n_hits, hits, m_diags.begin(), interval );

    // now sort the results by diagonal (which can be either a uint32 or a uint2)
    typedef typename if_equal<diagonal_type, uint32, uint32, uint64>::type primitive_type;

    primitive_type* raw_diags( (primitive_type*)nvbio::raw_pointer( m_diags ) );
    thrust::sort(
        raw_diags,
        raw_diags + n_hits );

    // and run-length encode them
    const uint32 n_merged = uint32( thrust::reduce_by_key(
            m_diags.begin(),
            m_diags.begin() + n_hits,
            thrust::make_constant_iterator<uint32>(1u),
            merged_hits,
            merged_counts ).first - merged_hits );

    return n_merged;
}

// enact the q-gram filter
//
// \param qgram_index      the q-gram index
// \param n_queries        the number of query q-grams
// \param queries          the query q-grams
// \param indices          the query indices
//
template <typename qgram_index_type, typename query_iterator, typename index_iterator>
uint64 QGramFilter<device_tag, qgram_index_type, query_iterator, index_iterator>::rank(
    const qgram_index_type& qgram_index,
    const uint32            n_queries,
    const query_iterator    queries,
    const index_iterator    indices)
{
    typedef typename qgram_index_type::coord_type coord_type;

    // save the query
    m_n_queries   = n_queries;
    m_queries     = queries;
    m_indices     = indices;
    m_qgram_index = nvbio::plain_view( qgram_index );

    // alloc enough storage for the results
    if (m_ranges.size() < n_queries)
    {
        m_ranges.clear();
        m_ranges.resize( n_queries );
    }
    if (m_slots.size() < n_queries)
    {
        m_slots.clear();
        m_slots.resize( n_queries );
    }

    // search the q-grams in the index, obtaining a set of ranges
    thrust::transform(
        device_iterator( queries ),
        device_iterator( queries ) + n_queries,
        m_ranges.begin(),
        m_qgram_index );

    // scan their size to determine the slots
    cuda::inclusive_scan(
        n_queries,
        thrust::make_transform_iterator( m_ranges.begin(), qgram::range_size() ),
        m_slots.begin(),
        thrust::plus<uint64>(),
        d_temp_storage );

    // determine the total number of occurrences
    m_n_occurrences = m_slots[ n_queries-1 ];
    return m_n_occurrences;
}

// enumerate all hits in a given range
//
template <typename qgram_index_type, typename query_iterator, typename index_iterator>
template <typename hits_iterator>
void QGramFilter<device_tag, qgram_index_type, query_iterator, index_iterator>::locate(
    const uint64            begin,
    const uint64            end,
    hits_iterator           hits)
{
    typedef typename qgram_index_type::coord_type coord_type;

    // and fill it
    thrust::transform(
        thrust::make_counting_iterator<uint64>(0u) + begin,
        thrust::make_counting_iterator<uint64>(0u) + end,
        device_iterator( hits ),
        qgram::filter_results<qgram_index_view,index_iterator,coord_type>(
            m_qgram_index,
            m_n_queries,
            nvbio::plain_view( m_slots ),
            nvbio::plain_view( m_ranges ),
            m_indices ) );
}

// simply convert hits to diagonal coordinates
//
// \tparam hits_iterator         a hit_iterator iterator
// \tparam output_iterator       a diagonal_type iterator
//
// \param  n_hits          the number of input hits
// \param  hits            the input hits
// \param  diags           the output diagonals
//
template <typename qgram_index_type, typename query_iterator, typename index_iterator>
template <typename hits_iterator, typename output_iterator>
void QGramFilter<device_tag, qgram_index_type, query_iterator, index_iterator>::diagonals(
    const uint32            n_hits,
    const hits_iterator     hits,
          output_iterator   diags,
    const uint32            interval)
{
    // snap the diagonals to the closest one
    thrust::transform(
        device_iterator( hits ),
        device_iterator( hits ) + n_hits,
        device_iterator( diags ),
        qgram::closest_diagonal<hit_type>( interval ) );
}

// merge hits falling within the same diagonal interval; this method will
// replace the vector of hits with a compacted list of hits snapped to the
// closest sample diagonal (i.e. multiple of the given interval), together
// with a counts vector providing the number of hits falling on the same
// spot
//
// \param  interval        the merging interval
// \param  n_hits          the number of input hits
// \param  hits            the input hits
// \param  merged_hits     the output merged hits
// \param  merged_counts   the output merged counts
// \return                 the number of merged hits
//
template <typename qgram_index_type, typename query_iterator, typename index_iterator>
template <typename hits_iterator, typename output_iterator, typename count_iterator>
uint32 QGramFilter<device_tag, qgram_index_type, query_iterator, index_iterator>::merge(
    const uint32            interval,
    const uint32            n_hits,
    const hits_iterator     hits,
          output_iterator   merged_hits,
          count_iterator    merged_counts)
{
    // copy the hits to a temporary sorting buffer
    const uint32 buffer_size = align<32>( n_hits );
    m_diags.resize( buffer_size * 2u );

    // convert hits to diagonals and snap them to the closest one
    diagonals( n_hits, hits, m_diags.begin(), interval );

    // now sort the results by diagonal (which can be either a uint32 or a uint2)
    typedef typename if_equal<diagonal_type, uint32, uint32, uint64>::type primitive_type;

    primitive_type* raw_diags( (primitive_type*)nvbio::raw_pointer( m_diags ) );

    cuda::SortBuffers<primitive_type*> sort_buffers;
    sort_buffers.keys[0] = raw_diags;
    sort_buffers.keys[1] = raw_diags + buffer_size;

    cuda::SortEnactor sort_enactor;
    sort_enactor.sort( n_hits, sort_buffers );

    // and run-length encode them
    const uint32 n_merged = cuda::runlength_encode(
        n_hits,
        m_diags.begin() + sort_buffers.selector  * buffer_size,
        merged_hits,
        merged_counts,
        d_temp_storage );

    return n_merged;
}

} // namespace nvbio
