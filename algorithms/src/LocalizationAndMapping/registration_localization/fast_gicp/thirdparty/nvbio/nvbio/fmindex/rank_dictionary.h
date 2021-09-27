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

#include <nvbio/basic/types.h>
#include <nvbio/basic/numbers.h>
#include <nvbio/basic/popcount.h>
#include <nvbio/basic/packedstream.h>
#include <nvbio/basic/iterator.h>
#include <nvbio/basic/static_vector.h>
#include <vector_types.h>
#include <vector_functions.h>

namespace nvbio {

///@addtogroup FMIndex
///@{

///\defgroup RankDictionaryModule Rank Dictionaries
///
/// A rank dictionary is a data-structure which, given a text and a sparse occurrence table, can answer,
/// in O(1) time, queries of the kind "how many times does character c occurr in the substring text[0:i] ?"
///\par
/// <i>NOTE:</i>  the O(1) time refers to the complexity in the length <i>n</i> of the text; more properly, if the
/// alphabet contains <i>s</i> characters (i.e. if the number of bits per symbol is b = log(s)), the complexity
/// is O(log(s)).
/// Similarly, the amount of space needed for a sparse occurrence table is O(n s).
/// In other words, the amount of space is exponential in the number of bits per character.
///\par
/// For a more compact data structure requiring O(n log(s)) storage, useful with larger alphabets, please
/// refer to \ref WaveletTreeSection.
///

///@addtogroup RankDictionaryModule
///@{

///
/// A rank dictionary data-structure which, given a text and a sparse occurrence table, can answer,
/// in O(1) time, queries of the kind "how many times does character c occurr in the substring text[0:i] ?"
///\par
/// <i>NOTE:</i>  the O(1) time refers to the complexity in the length of the text; more properly, if the
/// alphabet contains <i>s</i> characters (i.e. if the number of bits per symbol is b = log(s)), the complexity
/// is O(log(s)).
/// Similarly, the amount of space needed for a sparse occurrence table is O(n s).
/// In other words, the amount of space needed is exponential in the number of bits per character.
///\par
/// For a more compact data structure requiring O(n log(s)) storage, useful with larger alphabets, please
/// refer to \ref WaveletTreeSection.
///
/// \tparam SYMBOL_SIZE_T       the size of the alphabet, in bits
/// \tparam K                   the sparsity of the occurrence table
/// \tparam TextString          the text string type
/// \tparam OccIterator         the occurrence table iterator type
/// \tparam CountTable          an auxiliary lookup table used to count the number of occurrences of all
///                             characters in a given byte
///
template <uint32 SYMBOL_SIZE_T, uint32 K, typename TextString, typename OccIterator, typename CountTable>
struct rank_dictionary
{
    static const uint32     BLOCK_INTERVAL  = K;
    static const uint32     SYMBOL_SIZE     = SYMBOL_SIZE_T;
    static const uint32     SYMBOL_COUNT    = 1u << SYMBOL_SIZE;

    typedef TextString      text_type;
    typedef OccIterator     occ_iterator;
    typedef CountTable      count_table_type;

    // the indexing type of this container is determined by the value_type of the occurrence table
    typedef typename vector_traits<
        typename std::iterator_traits<occ_iterator>::value_type>::value_type index_type;

    typedef typename vector_type<index_type,2>::type                range_type;
    typedef typename vector_type<index_type,2>::type                vec2_type;
    typedef typename vector_type<index_type,4>::type                vec4_type;
    typedef StaticVector<index_type,SYMBOL_COUNT>                   vector_type;

    /// default constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    rank_dictionary() {}

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    rank_dictionary(
        const TextString   _text,
        const OccIterator  _occ,
        const CountTable   _count_table) :
        m_text( _text ),
        m_occ( _occ ),
        m_count_table( _count_table ) {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 symbol_count() const { return 1u << SYMBOL_SIZE_T; }
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 symbol_size()  const { return SYMBOL_SIZE_T; }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE text_type text()       { return m_text; }
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE text_type text() const { return m_text; }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE occ_iterator occ()       { return m_occ; }
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE occ_iterator occ() const { return m_occ; }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE count_table_type count_table()       { return m_count_table; }
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE count_table_type count_table() const { return m_count_table; }

    TextString    m_text;                   ///< the dictionary's text
    OccIterator   m_occ;                    ///< the dictionary's occurrence table
    CountTable    m_count_table;            ///< a helper lookup table used to efficiently count the number
                                          ///  of occurrences of all the characters in a given byte
};

///
/// \relates rank_dictionary
///
/// Build a sampled occurrence table for a given string, storing a set of symbol counters
/// every K elements of the original string.
/// The table must contain ((n+K-1)/K)*N_SYMBOLS entries, where N_SYMBOLS = 2 ^ SYMBOL_SIZE.
///
/// \tparam SYMBOL_SIZE         symbol size, in bits
/// \tparam K                   sampling frequency
/// \tparam SymbolIterator      the input string iterator
/// \tparam IndexType           the integer type used to store indices
///
/// Optionally save the table of the global counters as well.
///
/// \param begin    symbol sequence begin
/// \param end      symbol sequence end
/// \param occ      output occurrence map
/// \param cnt      optional table of the global counters
///
template <uint32 SYMBOL_SIZE, uint32 K, typename SymbolIterator, typename IndexType>
void build_occurrence_table(
    SymbolIterator begin,
    SymbolIterator end,
    IndexType*     occ,
    IndexType*     cnt = NULL);

/// \relates rank_dictionary
/// fetch the text character at position i in the rank dictionary
///
template <uint32 SYMBOL_SIZE_T, uint32 K, typename TextString, typename OccIterator, typename CountTable>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint8 text(const rank_dictionary<SYMBOL_SIZE_T,K,TextString,OccIterator,CountTable>& dict, const uint32 i);

/// \relates rank_dictionary
/// fetch the text character at position i in the rank dictionary
///
template <uint32 SYMBOL_SIZE_T, uint32 K, typename TextString, typename OccIterator, typename CountTable>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint8 text(const rank_dictionary<SYMBOL_SIZE_T,K,TextString,OccIterator,CountTable>& dict, const uint64 i);

/// \relates rank_dictionary
/// fetch the number of occurrences of character c in the substring [0,i]
///
/// \param dict         the rank dictionary
/// \param i            the end of the query range [0,i]
/// \param c            the query character
///
template <uint32 SYMBOL_SIZE_T, uint32 K, typename TextString, typename OccIterator, typename CountTable, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE IndexType rank(
    const rank_dictionary<SYMBOL_SIZE_T,K,TextString,OccIterator,CountTable>& dict, const IndexType i, const uint32 c);

/// \relates rank_dictionary
/// fetch the number of occurrences of character c in the substrings [0,l] and [0,r]
///
/// \param dict         the rank dictionary
/// \param range        the ends of the query ranges [0,range.x] and [0,range.y]
/// \param c            the query character
///
template <uint32 SYMBOL_SIZE_T, uint32 K, typename TextString, typename OccIterator, typename CountTable, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE typename vector_type<IndexType,2>::type rank(
    const rank_dictionary<SYMBOL_SIZE_T,K,TextString,OccIterator,CountTable>& dict, const typename vector_type<IndexType,2>::type range, const uint32 c);

/// \relates rank_dictionary
/// fetch the number of occurrences of all characters c in the substring [0,i]
///
/// \param dict         the rank dictionary
/// \param i            the end of the query range [0,i]
///
/// this function is <b>deprecated</b>: please use rank_all()
///
template <uint32 K, typename TextString, typename OccIterator, typename CountTable, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE typename vector_type<IndexType,4>::type rank4(
    const rank_dictionary<2,K,TextString,OccIterator,CountTable>& dict, const IndexType i);

/// \relates rank_dictionary
/// fetch the number of occurrences of all characters in the substrings [0,l] and [0,r]
///
/// \param dict         the rank dictionary
/// \param range        the ends of the query ranges [0,range.x] and [0,range.y]
/// \param outl         the output count of all characters in the first range
/// \param outl         the output count of all characters in the second range
///
/// this function is <b>deprecated</b>: please use rank_all()
///
template <uint32 K, typename TextString, typename OccIterator, typename CountTable>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE void rank4(
    const rank_dictionary<2,K,TextString,OccIterator,CountTable>& dict, const uint2 range, uint4* outl, uint4* outh);

/// \relates rank_dictionary
/// fetch the number of occurrences of all characters in the substrings [0,l] and [0,r]
///
/// \param dict         the rank dictionary
/// \param range        the ends of the query ranges [0,range.x] and [0,range.y]
/// \param outl         the output count of all characters in the first range
/// \param outl         the output count of all characters in the second range
///
/// this function is <b>deprecated</b>: please use rank_all()
///
template <uint32 K, typename TextString, typename OccIterator, typename CountTable>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE void rank4(
    const rank_dictionary<2,K,TextString,OccIterator,CountTable>& dict, const uint64_2 range, uint64_4* outl, uint64_4* outh);

/// \relates rank_dictionary
/// fetch the number of occurrences of all characters c in the substring [0,i]
///
/// \param dict         the rank dictionary
/// \param i            the end of the query range [0,i]
///
template <uint32 SYMBOL_SIZE, uint32 K, typename TextString, typename OccIterator, typename CountTable>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
typename rank_dictionary<SYMBOL_SIZE,K,TextString,OccIterator,CountTable>::vector_type
rank_all(
    const rank_dictionary<SYMBOL_SIZE,K,TextString,OccIterator,CountTable>&                         dict,
    const typename rank_dictionary<SYMBOL_SIZE,K,TextString,OccIterator,CountTable>::index_type     i);

/// \relates rank_dictionary
/// fetch the number of occurrences of all characters c in the substring [0,i]
///
/// \param dict         the rank dictionary
/// \param i            the end of the query range [0,i]
/// \param out          the output count of all characters
///
template <uint32 SYMBOL_SIZE, uint32 K, typename TextString, typename OccIterator, typename CountTable>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void rank_all(
    const rank_dictionary<SYMBOL_SIZE,K,TextString,OccIterator,CountTable>&                         dict,
    const typename rank_dictionary<SYMBOL_SIZE,K,TextString,OccIterator,CountTable>::index_type     i,
          typename rank_dictionary<SYMBOL_SIZE,K,TextString,OccIterator,CountTable>::vector_type*   out);

/// \relates rank_dictionary
/// fetch the number of occurrences of all characters in the substrings [0,l] and [0,r]
///
/// \param dict         the rank dictionary
/// \param range        the ends of the query ranges [0,range.x] and [0,range.y]
/// \param outl         the output count of all characters in the first range
/// \param outl         the output count of all characters in the second range
///
template <uint32 SYMBOL_SIZE, uint32 K, typename TextString, typename OccIterator, typename CountTable>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void rank_all(
    const rank_dictionary<SYMBOL_SIZE,K,TextString,OccIterator,CountTable>&                         dict,
    const typename rank_dictionary<SYMBOL_SIZE,K,TextString,OccIterator,CountTable>::range_type     range,
          typename rank_dictionary<SYMBOL_SIZE,K,TextString,OccIterator,CountTable>::vector_type*   outl,
          typename rank_dictionary<SYMBOL_SIZE,K,TextString,OccIterator,CountTable>::vector_type*   outh);

///@} RankDictionaryModule
///@} FMIndex

} // namespace nvbio

#include <nvbio/fmindex/rank_dictionary_inl.h>
