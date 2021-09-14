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

namespace fmindex {

// return the size of a given range
template <typename range_type>
struct range_size
{
    typedef range_type argument_type;
    typedef uint64     result_type;

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint64 operator() (const range_type range) const { return 1u + range.y - range.x; }
};

template <typename index_type, typename string_set_type>
struct rank_functor
{
    typedef typename index_type::range_type range_type;

    typedef uint32                          argument_type;
    typedef range_type                      result_type;

    // constructor
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    rank_functor(
        const index_type        _index,
        const string_set_type   _string_set) :
    index       ( _index ),
    string_set  ( _string_set ) {}

    // functor operator
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    result_type operator() (const argument_type string_id) const
    {
        typedef typename string_set_type::string_type   string_type;

        // fetch the given string
        const string_type string = string_set[ string_id ];

        // and match it in the FM-index
        return match( index, string, length( string ) );
    }

    const index_type        index;
    const string_set_type   string_set;
};

template <typename range_type>
struct filter_results
{
    typedef typename vector_traits<range_type>::value_type  coord_type;

    typedef uint64      argument_type;
    typedef range_type  result_type;

    // constructor
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    filter_results(
        const uint32        _n_queries,
        const uint64*       _slots,
        const range_type*   _ranges) :
    n_queries   ( _n_queries ),
    slots       ( _slots ),
    ranges      ( _ranges ) {}

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
        const uint32 string_id   = slot;

        // locate the hit position
        const range_type range   = ranges[ slot ];
        const uint64 base_slot   = slot ? slots[ slot-1 ] : 0u;
        const uint32 local_index = output_index - base_slot;

        // and write out the pair (qgram_pos,text_pos)
        return make_vector( coord_type( range.x + local_index ), coord_type( string_id ) );
    }

    const uint32        n_queries;
    const uint64*       slots;
    const range_type*   ranges;
};

template <typename index_type>
struct locate_results
{
    typedef typename index_type::range_type range_type;

    typedef range_type   argument_type;
    typedef range_type   result_type;

    // constructor
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    locate_results(const index_type _index) : index( _index ) {}

    // functor operator
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    result_type operator() (const range_type pair) const
    {
        return make_vector( locate( index, pair.x ), pair.y );
    }

    const index_type index;
};

template <typename index_type>
struct locate_ssa_results
{
    typedef typename index_type::range_type range_type;

    typedef range_type   argument_type;
    typedef range_type   result_type;

    // constructor
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    locate_ssa_results(const index_type _index) : index( _index ) {}

    // functor operator
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    result_type operator() (const range_type pair) const
    {
        return locate_ssa_iterator( index, pair.x );
    }

    const index_type index;
};

template <typename index_type>
struct lookup_ssa_results
{
    typedef typename index_type::range_type range_type;

    typedef range_type   first_argument_type;
    typedef range_type   second_argument_type;
    typedef range_type   result_type;           // TODO: this should be the filter's hit_type

    // constructor
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    lookup_ssa_results(const index_type _index) : index( _index ) {}

    // functor operator
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    result_type operator() (const range_type pair, const range_type ssa) const
    {
        return make_vector( lookup_ssa_iterator( index, ssa ), pair.y );
    }

    const index_type index;
};

} // namespace fmindex


// enact the filter on an FM-index and a string-set
//
// \param fm_index         the FM-index
// \param string-set       the query string-set
//
// \return the total number of hits
//
template <typename fm_index_type>
template <typename string_set_type>
uint64 FMIndexFilter<host_tag, fm_index_type>::rank(
    const fm_index_type&    index,
    const string_set_type&  string_set)
{
    // save the query
    m_n_queries   = string_set.size();
    m_index       = index;

    // alloc enough storage for the results
    m_ranges.resize( m_n_queries );
    m_slots.resize( m_n_queries );

    // search the strings in the index, obtaining a set of ranges
    thrust::transform(
        thrust::make_counting_iterator<uint32>(0u),
        thrust::make_counting_iterator<uint32>(0u) + m_n_queries,
        m_ranges.begin(),
        fmindex::rank_functor<fm_index_type,string_set_type>( m_index, string_set ) );

    // scan their size to determine the slots
    thrust::inclusive_scan(
        thrust::make_transform_iterator( m_ranges.begin(), fmindex::range_size<range_type>() ),
        thrust::make_transform_iterator( m_ranges.begin(), fmindex::range_size<range_type>() ) + m_n_queries,
        m_slots.begin() );

    // determine the total number of occurrences
    m_n_occurrences = m_slots[ m_n_queries-1 ];
    return m_n_occurrences;
}

// enumerate all hits in a given range
//
// \tparam hits_iterator         a hit_type iterator
//
template <typename fm_index_type>
template <typename hits_iterator>
void FMIndexFilter<host_tag,fm_index_type>::locate(
    const uint64    begin,
    const uint64    end,
    hits_iterator   hits)
{
    // fill the output hits with (SA,string-id) coordinates
    thrust::transform(
        thrust::make_counting_iterator<uint64>(0u) + begin,
        thrust::make_counting_iterator<uint64>(0u) + end,
        hits,
        fmindex::filter_results<range_type>(
            m_n_queries,
            nvbio::plain_view( m_slots ),
            nvbio::plain_view( m_ranges ) ) );

    // and locate the SA coordinates
    thrust::transform(
        hits,
        hits + (end - begin),
        hits,
        fmindex::locate_results<fm_index_type>( m_index ) );
}

// enact the filter on an FM-index and a string-set
//
// \param fm_index         the FM-index
// \param string-set       the query string-set
//
// \return the total number of hits
//
template <typename fm_index_type>
template <typename string_set_type>
uint64 FMIndexFilter<device_tag,fm_index_type>::rank(
    const fm_index_type&    index,
    const string_set_type&  string_set)
{
    // save the query
    m_n_queries   = string_set.size();
    m_index       = index;

    // alloc enough storage for the results
    m_ranges.resize( m_n_queries );
    m_slots.resize( m_n_queries );

    // search the strings in the index, obtaining a set of ranges
    thrust::transform(
        thrust::make_counting_iterator<uint32>(0u),
        thrust::make_counting_iterator<uint32>(0u) + m_n_queries,
        m_ranges.begin(),
        fmindex::rank_functor<fm_index_type,string_set_type>( m_index, string_set ) );

    // scan their size to determine the slots
    cuda::inclusive_scan(
        m_n_queries,
        thrust::make_transform_iterator( m_ranges.begin(), fmindex::range_size<range_type>() ),
        m_slots.begin(),
        thrust::plus<uint64>(),
        d_temp_storage );

    // determine the total number of occurrences
    m_n_occurrences = m_slots[ m_n_queries-1 ];
    return m_n_occurrences;
}

// enumerate all hits in a given range
//
// \tparam hits_iterator         a hit_type iterator
//
template <typename fm_index_type>
template <typename hits_iterator>
void FMIndexFilter<device_tag,fm_index_type>::locate(
    const uint64    begin,
    const uint64    end,
    hits_iterator   hits)
{
#if 0
    const uint32 n_hits = end - begin;
    const uint32 buffer_size = align<32>( n_hits );

    if (m_hits.size() < buffer_size * 2u)
    {
        m_hits.clear();
        m_hits.resize( buffer_size * 2u );
    }

    // fill the output hits with (SA,string-id) coordinates
    thrust::transform(
        thrust::make_counting_iterator<uint64>(0u) + begin,
        thrust::make_counting_iterator<uint64>(0u) + end,
        m_hits.begin(),
        fmindex::filter_results<range_type>(
            m_n_queries,
            nvbio::plain_view( m_slots ),
            nvbio::plain_view( m_ranges ) ) );

    // sort by the first 8 bits of the SA coordinates
    uint64* raw_hits( (uint64*)nvbio::plain_view( m_hits ) );

    cuda::SortBuffers<uint64*> sort_buffers;
    sort_buffers.keys[0] = raw_hits;
    sort_buffers.keys[1] = raw_hits + buffer_size;

    cuda::SortEnactor sort_enactor;
    sort_enactor.sort( n_hits, sort_buffers, 0u, 8u );

    const uint32 pairs_selector = sort_buffers.selector;
    const uint32 ssa_selector   = sort_buffers.selector ? 0u : 1u;

    // locate the SSA iterators
    thrust::transform(
        m_hits.begin() + buffer_size * pairs_selector,
        m_hits.begin() + buffer_size * pairs_selector + n_hits,
        m_hits.begin() + buffer_size * ssa_selector,
        fmindex::locate_ssa_results<fm_index_type>( m_index ) );

    // perform the final SSA lookup
    thrust::transform(
        m_hits.begin() + buffer_size * pairs_selector,
        m_hits.begin() + buffer_size * pairs_selector + n_hits,
        m_hits.begin() + buffer_size * ssa_selector,
        m_hits.begin() + buffer_size * pairs_selector,
        fmindex::lookup_ssa_results<fm_index_type>( m_index ) );

    // and sort back by string-id into final position
    sort_enactor.sort( n_hits, sort_buffers, 32u, 64u );

    thrust::copy(
        m_hits.begin() + buffer_size * sort_buffers.selector,
        m_hits.begin() + buffer_size * sort_buffers.selector + n_hits,
        device_iterator( hits ) );
#else
    const uint32 n_hits = end - begin;

    if (m_hits.size() < n_hits)
    {
        m_hits.clear();
        m_hits.resize( n_hits );
    }

    // fill the output hits with (SA,string-id) coordinates
    thrust::transform(
        thrust::make_counting_iterator<uint64>(0u) + begin,
        thrust::make_counting_iterator<uint64>(0u) + end,
        device_iterator( hits ),
        fmindex::filter_results<range_type>(
            m_n_queries,
            nvbio::plain_view( m_slots ),
            nvbio::plain_view( m_ranges ) ) );

    // locate the SSA iterators
    thrust::transform(
        device_iterator( hits ),
        device_iterator( hits ) + n_hits,
        m_hits.begin(),
        fmindex::locate_ssa_results<fm_index_type>( m_index ) );

    // and perform the final SSA lookup
    thrust::transform(
        device_iterator( hits ),
        device_iterator( hits ) + n_hits,
        m_hits.begin(),
        device_iterator( hits ),
        fmindex::lookup_ssa_results<fm_index_type>( m_index ) );
#endif
}

} // namespace nvbio
