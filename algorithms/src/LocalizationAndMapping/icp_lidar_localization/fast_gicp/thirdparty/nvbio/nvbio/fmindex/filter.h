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

#include <nvbio/fmindex/fmindex.h>
#include <nvbio/basic/types.h>
#include <nvbio/basic/numbers.h>
#include <nvbio/basic/algorithms.h>
#include <nvbio/basic/exceptions.h>
#include <nvbio/basic/vector.h>
#include <nvbio/basic/cuda/sort.h>
#include <nvbio/basic/cuda/primitives.h>
#include <nvbio/strings/string.h>
#include <thrust/sort.h>
#include <thrust/scan.h>
#include <thrust/iterator/constant_iterator.h>
#include <thrust/iterator/counting_iterator.h>

namespace nvbio {

///@addtogroup FMIndex
///@{

///
///\par
/// This class implements a FM-index filter which can be used to find and filter matches
/// between an arbitrary string-set and an \ref FMIndex "FM-index".
///\par
/// The filter will return an ordered set of <i>(index-pos,string-id)</i> pairs, where <i>string-id</i> is
/// the index into the string-set and <i>index-pos</i> is an index into the FM-index.
///\par
///
/// \tparam fm_index_type    the type of the fm-index
///
template <typename system_tag, typename fm_index_type>
struct FMIndexFilter {};

///
///\par
/// This class implements a FM-index filter which can be used to find and filter matches
/// between an arbitrary string-set and an \ref FMIndex "FM-index".
///\par
/// The filter will return an ordered set of <i>(index-pos,string-id)</i> pairs, where <i>string-id</i> is
/// the index into the string-set and <i>index-pos</i> is an index into the FM-index.
///\par
///
/// \tparam fm_index_type    the type of the fm-index
///
template <typename fm_index_type>
struct FMIndexFilter<host_tag, fm_index_type>
{
    typedef host_tag                                        system_tag;     ///< the backend system
    typedef fm_index_type                                   index_type;     ///< the index type

    typedef typename index_type::index_type                 coord_type;     ///< the coordinate type of the fm-index, uint32|uint64|uint32_2|uint64_2
    static const uint32                                     coord_dim = vector_traits<coord_type>::DIM;

    typedef typename vector_type<coord_type,2>::type        range_type;     ///< ranges are either uint32_2 or uint64_2;

    static const uint32                                     hit_dim = coord_dim*2;  ///< hits are either uint2 or uint4
    typedef typename vector_type<coord_type,hit_dim>::type  hit_type;               ///< hits are either uint2 or uint4

    /// enact the filter on an FM-index and a string-set
    ///
    /// \param index            the FM-index
    /// \param string-set       the query string-set
    ///
    /// \return the total number of hits
    ///
    template <typename string_set_type>
    uint64 rank(
        const fm_index_type&    index,
        const string_set_type&  string_set);

    /// enumerate all hits in a given range
    ///
    /// \tparam hits_iterator         a hit_type iterator
    ///
    /// \param begin                  the beginning of the hits sequence to locate, in [0,n_hits)
    /// \param end                    the end of the hits sequence to locate, in [0,n_hits]
    ///
    template <typename hits_iterator>
    void locate(
        const uint64    begin,
        const uint64    end,
        hits_iterator   hits);

    /// return the number of hits from the last rank query
    ///
    uint64 n_hits() const { return m_n_occurrences; }

    /// return the individual ranges of the ranked queries
    ///
    const range_type* ranges() const { return nvbio::plain_view( m_ranges ); }

    /// return the global ranks of the output hits (i.e. the range <i>[ranks[i], ranks[i+1])</i>
    /// identifies the position of the hits corresponding to the i-th query in the locate output)
    ///
    const uint64* ranks() const { return nvbio::plain_view( m_slots ); }

    uint32                              m_n_queries;
    index_type                          m_index;
    uint64                              m_n_occurrences;
    thrust::host_vector<range_type>     m_ranges;
    thrust::host_vector<uint64>         m_slots;
};

///
///\par
/// This class implements a FM-index filter which can be used to find and filter matches
/// between an arbitrary string-set and an \ref FMIndex "FM-index".
///\par
/// The filter will return an ordered set of <i>(index-pos,string-id)</i> pairs, where <i>string-id</i> is
/// the index into the string-set and <i>index-pos</i> is an index into the FM-index.
///\par
///
/// \tparam fm_index_type    the type of the fm-index
///
template <typename fm_index_type>
struct FMIndexFilter<device_tag, fm_index_type>
{
    typedef device_tag                                      system_tag;     ///< the backend system
    typedef fm_index_type                                   index_type;     ///< the index type

    typedef typename index_type::index_type                 coord_type;     ///< the coordinate type of the fm-index, uint32|uint64|uint32_2|uint64_2
    static const uint32                                     coord_dim = vector_traits<coord_type>::DIM;

    typedef typename vector_type<coord_type,2>::type        range_type;     ///< ranges are either uint32_2 or uint64_2;

    static const uint32                                     hit_dim = coord_dim*2;  ///< hits are either uint2 or uint4
    typedef typename vector_type<coord_type,hit_dim>::type  hit_type;               ///< hits are either uint2 or uint4

    /// enact the filter on an FM-index and a string-set
    ///
    /// \param index            the FM-index
    /// \param string-set       the query string-set
    ///
    /// \return the total number of hits
    ///
    template <typename string_set_type>
    uint64 rank(
        const fm_index_type&    index,
        const string_set_type&  string_set);

    /// enumerate all hits in a given range
    ///
    /// \tparam hits_iterator         a hit_type iterator
    ///
    /// \param begin                  the beginning of the hits sequence to locate, in [0,n_hits)
    /// \param end                    the end of the hits sequence to locate, in [0,n_hits]
    ///
    template <typename hits_iterator>
    void locate(
        const uint64    begin,
        const uint64    end,
        hits_iterator   hits);

    /// return the number of hits from the last rank query
    ///
    uint64 n_hits() const { return m_n_occurrences; }

    /// return the individual ranges of the ranked queries
    ///
    const range_type* ranges() const { return nvbio::plain_view( m_ranges ); }

    /// return the global ranks of the output hits (i.e. the range <i>[ranks[i], ranks[i+1])</i>
    /// identifies the position of the hits corresponding to the i-th query in the locate output)
    ///
    const uint64* ranks() const { return nvbio::plain_view( m_slots ); }

    uint32                              m_n_queries;
    index_type                          m_index;
    uint64                              m_n_occurrences;
    thrust::device_vector<range_type>   m_ranges;
    thrust::device_vector<uint64>       m_slots;
    thrust::device_vector<hit_type>     m_hits;
    thrust::device_vector<uint8>        d_temp_storage;
};

///
///\par
/// This class implements a FM-index filter which can be used to find and filter matches
/// between an arbitrary string-set and an \ref FMIndex "FM-index".
///\par
/// The filter will return an ordered set of <i>(index-pos,string-id)</i> pairs, where <i>string-id</i> is
/// the index into the string-set and <i>index-pos</i> is an index into the FM-index.
///\par
///
/// \tparam fm_index_type    the type of the fm-index
///
template <typename fm_index_type>
struct FMIndexFilterHost : public FMIndexFilter<host_tag, fm_index_type>
{
    typedef FMIndexFilter<host_tag, fm_index_type>          core_type;      ///< the base class
    typedef typename core_type::system_tag                  system_tag;     ///< the backend system
    typedef typename core_type::index_type                  index_type;     ///< the index type

    typedef typename core_type::coord_type                  coord_type;     ///< the coordinate type of the fm-index, uint32|uint2
    typedef typename core_type::range_type                  range_type;     ///< the coordinate type of the filtered ranges
    typedef typename core_type::hit_type                    hit_type;       ///< hits are either uint2 or uint4
};

///
///\par
/// This class implements a FM-index filter which can be used to find and filter matches
/// between an arbitrary string-set and an \ref FMIndex "FM-index".
///\par
/// The filter will return an ordered set of <i>(index-pos,string-id)</i> pairs, where <i>string-id</i> is
/// the index into the string-set and <i>index-pos</i> is an index into the FM-index.
///\par
///
/// \tparam fm_index_type    the type of the fm-index
///
template <typename fm_index_type>
struct FMIndexFilterDevice : public FMIndexFilter<device_tag, fm_index_type>
{
    typedef FMIndexFilter<device_tag, fm_index_type>        core_type;      ///< the base class
    typedef typename core_type::system_tag                  system_tag;     ///< the backend system
    typedef typename core_type::index_type                  index_type;     ///< the index type

    typedef typename core_type::coord_type                  coord_type;     ///< the coordinate type of the fm-index, uint32|uint2
    typedef typename core_type::range_type                  range_type;     ///< the coordinate type of the filtered ranges
    typedef typename core_type::hit_type                    hit_type;       ///< hits are either uint2 or uint4
};

///@} // end of the FMIndex group

} // namespace nvbio

#include <nvbio/fmindex/filter_inl.h>
