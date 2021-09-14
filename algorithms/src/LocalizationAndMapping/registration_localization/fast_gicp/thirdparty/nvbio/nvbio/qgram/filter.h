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

#include <nvbio/qgram/qgram.h>
#include <nvbio/basic/types.h>
#include <nvbio/basic/numbers.h>
#include <nvbio/basic/vector.h>
#include <nvbio/basic/algorithms.h>
#include <nvbio/basic/exceptions.h>
#include <nvbio/basic/cuda/sort.h>
#include <nvbio/basic/cuda/primitives.h>
#include <thrust/sort.h>
#include <thrust/scan.h>
#include <thrust/iterator/constant_iterator.h>
#include <thrust/iterator/counting_iterator.h>

namespace nvbio {

///@addtogroup QGram
///@{

///
/// This class implements a q-gram filter which can be used to find and filter matches
/// between an arbitrary set of indexed query q-grams, representing q-grams of a given
/// text, and a \ref QGramIndex "q-gram index".
/// The q-gram index can be either a simple string index or a string-set index.
///\par
/// For string q-gram indices, the filter will return an ordered set of <i>(qgram-pos,query-pos)</i>
/// pairs, where <i>qgram-pos</i> is the index of the hit into the string used to build qgram-index,
/// and <i>query-pos</i> corresponds to one of the input query q-gram indices.
///\par
/// For string-set q-gram indices, the filter will return an ordered set of <i>(string-id,string-pos,query-pos)</i>
/// pairs (encoded as uint4), where <i>string-id</i> is the index of the hit into the string-set used to build qgram-index,
/// and <i>query-pos</i> corresponds to one of the input query q-gram indices.
///
/// \tparam qgram_index_type    the type of the qgram-index
/// \tparam query_iterator      the type of the query q-gram iterator
/// \tparam index_iterator      the type of the query index iterator
///
template <typename system_tag, typename qgram_index_type, typename query_iterator, typename index_iterator>
struct QGramFilter {};


///
/// This class implements a q-gram filter which can be used to find and filter matches
/// between an arbitrary set of indexed query q-grams, representing q-grams of a given
/// text, and a \ref QGramIndex "q-gram index".
/// The q-gram index can be either a simple string index or a string-set index.
///\par
/// For string q-gram indices, the filter will return an ordered set of <i>(qgram-pos,query-pos)</i>
/// pairs, where <i>qgram-pos</i> is the index of the hit into the string used to build qgram-index,
/// and <i>query-pos</i> corresponds to one of the input query q-gram indices.
///\par
/// For string-set q-gram indices, the filter will return an ordered set of <i>(string-id,string-pos,query-pos)</i>
/// pairs (encoded as uint4), where <i>string-id</i> is the index of the hit into the string-set used to build qgram-index,
/// and <i>query-pos</i> corresponds to one of the input query q-gram indices.
///\par
/// Furthermore, the filter offers the ability to <i>merge</i> hits by diagonal bucket: in this case,
/// the output type will be either a simple uint32 linear coordinate describing the diagonal,
/// or a (string-id,diagonal) for string-set indices.
///
/// \tparam qgram_index_type    the type of the qgram-index
/// \tparam query_iterator      the type of the query q-gram iterator
/// \tparam index_iterator      the type of the query index iterator
///
template <typename qgram_index_type, typename query_iterator, typename index_iterator>
struct QGramFilter<host_tag, qgram_index_type, query_iterator, index_iterator>
{
    typedef host_tag            system_tag;

    typedef typename plain_view_subtype<const qgram_index_type>::type qgram_index_view;

    typedef typename qgram_index_type::coord_type coord_type;   ///< the coordinate type of the q-gram index, uint32|uint2

    /// the type of the hits will be either a uint2, if qgram_index_type is a string index,
    /// or a uint4 if it is a string-set index (the last entry of the uint4 is unused)
    ///
    typedef typename if_equal<coord_type, uint32, uint2,  uint4>::type  hit_type;
    typedef typename if_equal<coord_type, uint32, uint32, uint2>::type  diagonal_type;

    /// enact the q-gram filter on a q-gram index and a set of indexed query q-grams;\n
    /// <b>note:</b> the q-gram index and the query q-gram and index iterators will be
    /// cached inside the filter for any follow-up method calls (to e.g. locate() and
    /// merge())
    ///
    /// \param qgram_index      the q-gram index
    /// \param n_queries        the number of query q-grams
    /// \param queries          the query q-grams
    /// \param indices          the query indices
    ///
    /// \return the total number of hits
    ///
    uint64 rank(
        const qgram_index_type& qgram_index,
        const uint32            n_queries,
        const query_iterator    queries,
        const index_iterator    indices);

    /// enumerate all hits in a given range
    ///
    /// \tparam hits_iterator         a hit_type iterator
    ///
    template <typename hits_iterator>
    void locate(
        const uint64    begin,
        const uint64    end,
        hits_iterator   hits);

    /// simply convert hits to diagonal coordinates
    ///
    /// \tparam hits_iterator         a hit_iterator iterator
    /// \tparam output_iterator       a diagonal_type iterator
    ///
    /// \param  n_hits          the number of input hits
    /// \param  hits            the input hits
    /// \param  diags           the output diagonals
    /// \param  interval        the snapping interval
    ///
    template <typename hits_iterator, typename output_iterator>
    void diagonals(
        const uint32            n_hits,
        const hits_iterator     hits,
              output_iterator   diags,
        const uint32            interval = 1);

    /// merge hits falling within the same diagonal interval; given a vector of hits,
    /// this method will return a compacted list of hits snapped to the closest sample
    /// diagonal (i.e. multiple of the given interval), together with a counts vector
    /// providing their multiplicity, i.e. the number of hits falling on the same spot
    ///
    /// \tparam hits_iterator         a hit_iterator iterator
    /// \tparam output_iterator       a diagonal_type iterator
    /// \tparam counts_iterator       a uint8|uint16|uint32|uint64 iterator
    ///
    /// \param  interval        the merging interval
    /// \param  n_hits          the number of input hits
    /// \param  hits            the input hits
    /// \param  merged_hits     the output merged hits
    /// \param  merged_counts   the output merged counts
    /// \return                 the number of merged hits
    ///
    template <typename hits_iterator, typename output_iterator, typename count_iterator>
    uint32 merge(
        const uint32            interval,
        const uint32            n_hits,
        const hits_iterator     hits,
              output_iterator   merged_hits,
              count_iterator    merged_counts);

    /// return the individual ranges of the ranked queries
    ///
    const uint2* ranges() const { return nvbio::plain_view( m_ranges ); }

    /// return the global ranks of the output hits (i.e. the range <i>[ranks[i], ranks[i+1])</i>
    /// identifies the position of the hits corresponding to the i-th query in the locate output)
    ///
    const uint64* ranks() const { return nvbio::plain_view( m_slots ); }

    uint32                              m_n_queries;
    query_iterator                      m_queries;
    index_iterator                      m_indices;
    qgram_index_view                    m_qgram_index;
    uint64                              m_n_occurrences;
    thrust::host_vector<uint2>          m_ranges;
    thrust::host_vector<uint64>         m_slots;
    thrust::host_vector<diagonal_type>  m_diags;
};

////
/// This class implements a q-gram filter which can be used to find and filter matches
/// between an arbitrary set of indexed query q-grams, representing q-grams of a given
/// text, and a \ref QGramIndex "q-gram index".
/// The q-gram index can be either a simple string index or a string-set index.
///\par
/// For string q-gram indices, the filter will return an ordered set of <i>(qgram-pos,query-pos)</i>
/// pairs, where <i>qgram-pos</i> is the index of the hit into the string used to build qgram-index,
/// and <i>query-pos</i> corresponds to one of the input query q-gram indices.
///\par
/// For string-set q-gram indices, the filter will return an ordered set of <i>(string-id,string-pos,query-pos)</i>
/// pairs (encoded as uint4), where <i>string-id</i> is the index of the hit into the string-set used to build qgram-index,
/// and <i>query-pos</i> corresponds to one of the input query q-gram indices.
///\par
/// Furthermore, the filter offers the ability to <i>merge</i> hits by diagonal bucket: in this case,
/// the output type will be either a simple uint32 linear coordinate describing the diagonal,
/// or a (string-id,diagonal) for string-set indices.
///
template <typename qgram_index_type, typename query_iterator, typename index_iterator>
struct QGramFilter<device_tag, qgram_index_type, query_iterator, index_iterator>
{
    typedef device_tag          system_tag;

    typedef typename plain_view_subtype<const qgram_index_type>::type qgram_index_view;

    typedef typename qgram_index_type::coord_type coord_type;   ///< the coordinate type of the q-gram index, uint32|uint2

    /// the type of the hits will be either a uint2, if qgram_index_type is a string index,
    /// or a uint4 if it is a string-set index (the last entry of the uint4 is unused)
    ///
    typedef typename if_equal<coord_type, uint32, uint2,  uint4>::type  hit_type;
    typedef typename if_equal<coord_type, uint32, uint32, uint2>::type  diagonal_type;

    /// enact the q-gram filter on a q-gram index and a set of indexed query q-grams;\n
    /// <b>note:</b> the q-gram index and the query q-gram and index iterators will be
    /// cached inside the filter for any follow-up method calls (to e.g. locate() and
    /// merge())
    ///
    /// \param qgram_index      the q-gram index
    /// \param n_queries        the number of query q-grams
    /// \param queries          the query q-grams
    /// \param indices          the query indices
    ///
    uint64 rank(
        const qgram_index_type& qgram_index,
        const uint32            n_queries,
        const query_iterator    queries,
        const index_iterator    indices);

    /// enumerate all hits in a given range
    ///
    /// \typename hits_iterator         a uint2 iterator
    ///
    template <typename hits_iterator>
    void locate(
        const uint64            begin,
        const uint64            end,
        hits_iterator           hits);

    /// simply convert hits to diagonal coordinates
    ///
    /// \tparam hits_iterator         a hit_iterator iterator
    /// \tparam output_iterator       a diagonal_type iterator
    ///
    /// \param  n_hits          the number of input hits
    /// \param  hits            the input hits
    /// \param  diags           the output diagonals
    /// \param  interval        the snapping interval
    ///
    template <typename hits_iterator, typename output_iterator>
    void diagonals(
        const uint32            n_hits,
        const hits_iterator     hits,
              output_iterator   diags,
        const uint32            interval = 1);

    /// merge hits falling within the same diagonal interval; given a vector of hits,
    /// this method will return a compacted list of hits snapped to the closest sample
    /// diagonal (i.e. multiple of the given interval), together with a counts vector
    /// providing their multiplicity, i.e. the number of hits falling on the same spot
    ///
    /// \tparam hits_iterator         a hit_iterator iterator
    /// \tparam output_iterator       a diagonal_type iterator
    /// \tparam counts_iterator       a uint8|uint16|uint32|uint64 iterator
    ///
    /// \param  interval        the merging interval
    /// \param  n_hits          the number of input hits
    /// \param  hits            the input hits
    /// \param  merged_hits     the output merged hits
    /// \param  merged_counts   the output merged counts
    /// \return                 the number of merged hits
    ///
    template <typename hits_iterator, typename output_iterator, typename count_iterator>
    uint32 merge(
        const uint32            interval,
        const uint32            n_hits,
        const hits_iterator     hits,
              output_iterator   merged_hits,
              count_iterator    merged_counts);

    /// return the individual ranges of the ranked queries
    ///
    const uint2* ranges() const { return nvbio::plain_view( m_ranges ); }

    /// return the global ranks of the output hits (i.e. the range <i>[ranks[i], ranks[i+1])</i>
    /// identifies the position of the hits corresponding to the i-th query in the locate output)
    ///
    const uint64* ranks() const { return nvbio::plain_view( m_slots ); }

    uint32                                  m_n_queries;
    query_iterator                          m_queries;
    index_iterator                          m_indices;
    qgram_index_view                        m_qgram_index;
    uint64                                  m_n_occurrences;
    thrust::device_vector<uint2>            m_ranges;
    thrust::device_vector<uint64>           m_slots;
    thrust::device_vector<diagonal_type>    m_diags;
    thrust::device_vector<uint8>            d_temp_storage;
};

template <typename qgram_index_type, typename query_iterator, typename index_iterator>
struct QGramFilterHost : public QGramFilter<host_tag, qgram_index_type, query_iterator, index_iterator> {};

template <typename qgram_index_type, typename query_iterator, typename index_iterator>
struct QGramFilterDevice : public QGramFilter<device_tag, qgram_index_type, query_iterator, index_iterator> {};

///@} // end of the QGram group

} // namespace nvbio

#include <nvbio/qgram/filter_inl.h>
