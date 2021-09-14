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

#include <nvbio/strings/string.h>
#include <nvbio/basic/packedstream.h>
#include <nvbio/basic/vector_view.h>
#include <nvbio/basic/strided_iterator.h>
#include <nvbio/basic/cached_iterator.h>
#include <nvbio/basic/iterator.h>


namespace nvbio {

///\page strings_page Strings Module
///\par
/// This module provides generic constructs to work with strings and string-sets.
/// A string-set is a collection of strings. As there's many ways to encode a string,
/// there's even more ways to represent a string set.
/// For example, one might want to store the strings into a single concatenated array,
/// and store offsets to the beginning of each one.
/// Or he might want to build a small string set out of a sparse subset of a big string
/// (for example, to carve a few isolated regions out of a genome).
/// And the support string might be either encoded with ascii characters, or packed
/// using a few bits per symbol.
///
/// \section AlphabetsSection Alphabets
///\par
/// This module provides a set of operators to work with various alphabets:
///\par
/// - \ref Alphabet = { DNA, DNA_N, DNA_IUPAC, RNA, RNA_N, PROTEIN }
/// - \ref to_char
/// - \ref from_char
/// - \ref to_string
/// - \ref from_string
///\par
/// The list of all available operators is available in the \ref AlphabetsModule "Alphabet Module".
///
/// \section StringSetInterface String-Set Interfaces
///\par
/// String sets are generic, interchangeable containers that expose the same interface:
///\anchor StringSetAnchor
///\code
/// interface StringSet
/// {
///     // specify the type of the strings
///     typedef ... string_type;
///
///     // return the size of the set
///     uint32 size() const;
///
///     // return the i-th string in the set
///     string_type operator[] (const uint32) [const];
/// }
///\endcode
///\par
/// This module provides a few generic string set adaptors that can be "configured" by means of
/// the underlying template iterators.
/// The philosophy behind all of these containers is that they are just <i>shallow representations</i>,
/// holding no storage. Hence they can be used both in host and device code.
///\par
/// - ConcatenatedStringSet
/// - SparseStringSet
/// - StridedPackedStringSet
/// - StridedStringSet
/// - InfixSet
/// - PrefixSet
/// - SuffixSet
///\par
/// Furthermore, the module provides efficient generic copy() (resp. cuda::copy()) implementations to copy
/// a given host (resp. device) string set from a given layout into another with a different layout.
///
/// \section SeedingSection Seeding
///\par
/// Many bioinformatics applications need to extract short <i>seeds</i> out of strings and string-sets:
/// such seeds are nothing but \ref InfixSet "infixes". This module provides a few convenience functions to enumerate
/// all seeds resulting by applying a \ref SeedFunctor "Seeding Functor" to a string or string-set.
/// Internally, these simple functions employ massively parallel algorithms, which run either on the
/// host or on the bound cuda device, depending on whether the output is a host or device vector.
///\par
///\anchor SeedingAnchor
/// - enumerate_string_seeds()
/// - enumerate_string_set_seeds()
/// - uniform_seeds_functor
///
///\par
/// The following is a sample showing how to extract uniformly sampled seeds from a host or device string-set,
/// and construct an InfixSet to represent them:
///\code
/// // extract a set of uniformly spaced seeds from a string-set and return it as an InfixSet
/// //
/// template <typename system_tag, typename string_set_type>
/// InfixSet<string_set_type, const string_set_infix_coord_type*>
/// extract_seeds(
///     const string_set_type                                   string_set,
///     const uint32                                            seed_len,
///     const uint32                                            seed_interval,
///     nvbio::vector<system_tag,string_set_infix_coord_type>&  seed_coords)
/// {
///     // enumerate all seeds
///     const uint32 n_seeds = enumerate_string_set_seeds(
///         string_set,
///         uniform_seeds_functor<>( seed_len, seed_interval ),
///         seed_coords );
/// 
///     // and build the output infix-set
///     return InfixSet<string_set_type, const string_set_infix_coord_type*>(
///         n_seeds,
///         string_set,
///         nvbio::plain_view( seed_coords ) );
/// }
///\endcode
/// See examples/seeding/seeding.cu for more details.
///
/// \section WaveletTreeSection Wavelet Trees
///\par
/// A <i>Wavelet Tree</i> is a data structure that can be used to encode a string T of <i>n</i> symbols from an alphabet of <i>s</i>
/// characters in space O(n log(s)), allowing both symbol access and ranking in O(log(s)) time, i.e:
///\par
/// * each character T[i] can be recovered in O(log(s)) time
/// * the number of occurrences of a character c in the substring T[0,i] can be counted in O(log(s)) time
///\par
/// In other words, a Wavelet Tree is both an alternative string representation (often more amenable to compression),
/// <i>and</i> a storage-efficient \ref RankDictionarySection "rank dictionary".
/// For the sake of comparison, notice that the \ref rank_dictionary class, which is based on a standard sampled occurrence
/// table built directly on top of the original string T, needs O(s) storage - exponentially more in the number of bits
/// per symbol <i>b = log(s)</i>.
///\par
/// NVBIO provides two classes:
///\par
/// - WaveletTreeStorage : a proper wavelet tree implementation, which can instanced using either <i>host</i> or <i>device</i>
///   memory
/// - WaveletTree : a shallow, <i>storage-less</i> representation, which can be instanced on arbitrary user supplied iterators
///   to the actual data containers
///\par
/// as well as parallel host and device construction and lookup functions.
/// See the \ref WaveletTreeModule module for further documentation.
///\par
/// The following is a sample showing how to build a WaveletTree and query it:
///\code
/// void test_wavelet_tree()
/// {
///     const char* text = "This is a test String";
///     const uint32 text_len = uint32( nvbio::length( text ) );
///
///     // copy the text string onto the device, reinterpreting as uint8 as the WaveletTree
///     // expects symbols to be encoded using unsigned integral types
///     nvbio::vector<device_tag,uint8> d_text( text_len );
///     thrust::copy( (const uint8*)text, (const uint8*)text + text_len, d_text );
///
///     // instantiate a wavelet tree
///     WaveletTreeStorage<device_tag> wavelet_tree;
///
///     // setup the wavelet tree
///     setup( text_len, d_text.begin(), wavelet_tree );
///
///     // extract the text in parallel using the WaveletTree's unary functor operator()
///     // to transform the sequence of indices [0,text_len) into the sequence of
///     // corresponding characters
///     nvbio::transform<device_tag>(
///         text_len,                                       // number of characters to extract
///         thrust::make_counting_iterator<uint32>(0),      // list of character indices
///         d_text.begin(),                                 // output sequence
///         plain_view( wavelet_tree ) );                   // plain_view of the WaveletTreeStorage class
///
///     // copy the results back to the host
///     nvbio::vector<host_tag,uint8> h_extracted_text( d_text );
///
///     // check we extracted the right string...
///     printf("extracted \"%s\"", (const char*)raw_pointer( h_extracted_text ));
///
///     // check that in position 0 there should be exactly one 'T'
///     printf("rank(0,T) = %u\n", rank( plain_view( wavelet_tree ), 0, (uint8)'T' ));
///
///     // check that the range [0,6] contains exactly two 's'
///     printf("rank(6,s) = %u\n", rank( plain_view( wavelet_tree ), 6, (uint8)'s' ));
/// }
///\endcode
///
/// \section TechnicalOverviewSection Technical Overview
///\par
/// For a detailed description of all classes and functions see the \ref Strings documentation.
///

///\defgroup Strings Strings Module
///
/// This module defines various functions and classes to operate with strings and string-sets which vary
/// for the internal representation.
/// For a deeper explanation, see the \ref strings_page page.
///

///@addtogroup Strings
///@{

///\defgroup StringSetsModule String Sets
///
/// This module defines various types of string sets which vary for the internal representation.
/// For a deeper explanation, see the \ref string_page page.
///

///@addtogroup StringSetsModule
///@{

struct concatenated_string_set_tag {};
struct sparse_string_set_tag {};
struct strided_string_set_tag {};
struct strided_packed_string_set_tag {};

template <typename StringSetType>
struct StringSetIterator
{
    typedef typename StringSetType::system_tag                          system_tag;
    typedef typename StringSetType::string_type                         value_type;
    typedef typename StringSetType::string_type                         reference;
    typedef typename StringSetType::string_type*                        pointer;
    typedef int32                                                       difference_type;
    typedef typename if_equal<
        system_tag, host_tag,
        random_access_host_iterator_tag,
        random_access_device_iterator_tag>::type                        iterator_category;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE StringSetIterator() {}

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE StringSetIterator(StringSetType _string_set, const uint32 _idx) : string_set(_string_set), idx(_idx) {}

    /// indexing operator
    ///
    /// \param i        requested value
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE value_type operator[] (const uint32 i) const { return string_set[idx + i]; }

    /// indexing operator
    ///
    /// \param i        requested value
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE reference operator[] (const uint32 i) { return string_set[idx + i]; }

    /// dereference operator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE value_type operator* () const { return string_set[idx]; }

    /// dereference operator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE reference operator* () { return string_set[idx]; }

    StringSetType   string_set;
    uint32          idx;
};

/// difference operator
///
template <typename StringSet>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
typename StringSetIterator<StringSet>::difference_type operator- (
    const StringSetIterator<StringSet>& it1,
    const StringSetIterator<StringSet>& it2)
{
    return it1.idx - it2.idx;
}

/// operator+
///
template <typename StringSet>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
StringSetIterator<StringSet> operator+ (
    const StringSetIterator<StringSet>& it,
    const int32                         d)
{
    return StringSetIterator<StringSet>( it.string_set, it.idx + d );
}

/// operator-
///
template <typename StringSet>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
StringSetIterator<StringSet> operator- (
    const StringSetIterator<StringSet>& it,
    const int32                         d)
{
    return StringSetIterator<StringSet>( it.string_set, it.idx - d );
}

/// operator+=
///
template <typename StringSet>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
StringSetIterator<StringSet>& operator+= (
    StringSetIterator<StringSet>& it,
    const int32                   d)
{
    it.idx += d;
    return it;
}

/// operator-=
///
template <typename StringSet>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
StringSetIterator<StringSet>& operator-= (
    StringSetIterator<StringSet>& it,
    const int32                   d)
{
    it.idx -= d;
    return it;
}

/// operator<
///
template <typename StringSet>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool operator< (
    const StringSetIterator<StringSet>& it1,
    const StringSetIterator<StringSet>& it2)
{
    return it1.idx < it2.idx;
}

/// operator>
///
template <typename StringSet>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool operator> (
    const StringSetIterator<StringSet>& it1,
    const StringSetIterator<StringSet>& it2)
{
    return it1.idx > it2.idx;
}

/// operator<=
///
template <typename StringSet>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool operator<= (
    const StringSetIterator<StringSet>& it1,
    const StringSetIterator<StringSet>& it2)
{
    return it1.idx < it2.idx;
}

/// operator>=
///
template <typename StringSet>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool operator>= (
    const StringSetIterator<StringSet>& it1,
    const StringSetIterator<StringSet>& it2)
{
    return it1.idx >= it2.idx;
}

/// operator==
///
template <typename StringSet>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool operator== (
    const StringSetIterator<StringSet>& it1,
    const StringSetIterator<StringSet>& it2)
{
    return it1.idx == it2.idx;
}

/// operator!=
///
template <typename StringSet>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool operator!= (
    const StringSetIterator<StringSet>& it1,
    const StringSetIterator<StringSet>& it2)
{
    return it1.idx != it2.idx;
}

///
/// A "flat" collection of strings that are concatenated together into
/// a single one, and their starting points are given by a single offset vector.
///
/// Here's an example defining a simple concatenated string set:
///
///\code
/// // pack a few strings into a concatenated buffer
/// const uint32 n_strings = 3;
/// const char* strings = { "abc", "defghi", "lmno" };
///
///
/// thrust::host_vector<uint32> offsets_storage( n_strings+1 );
/// thrust::host_vector<char>   string_storage( strlen( strings[0] ) +
///                                             strlen( strings[1] ) +
///                                             strlen( strings[2] ) );
///
/// typedef ConcatenatedStringSet<char*, uint32*> string_set_type;
///
/// // build the string set
/// string_set_type string_set(
///     n_strings,
///     char*,
///     plain_view( offsets_storage ) );
///
/// // setup the offsets, note we need to place a sentinel
/// offsets_storage[0] = 0;
/// for (uint32 i = 0; i < n_reads; ++i)
///     offsets_storage[i+1] += strlen( strings[i] );
///
/// // and now we can conveniently access the i-th string of the set
/// for (uint32 i = 0; i < n_reads; ++i)
/// {
///     string_set_type::string_type string = string_set[i];
///
///     // and fill them in
///     for (uint32 j = 0; j < string.length(); ++j)
///         string[j] = strings[i][j];
/// }
///\endcode
///
/// or even a packed one:
///
/// \anchor ConcatenatedStringSetExample
///\code
/// // pack 1000 x 100bp reads into a single concatenated buffer
/// const uint32 n_reads    = 1000;
/// const uint32 read_len   = 100;
/// const uint32 n_words    = util::divide_ri( n_reads * read_len * 2, 32 );    // we can fit 16 bps in each word
///
/// thrust::host_vector<uint32> offsets_storage( n_reads+1 );
/// thrust::host_vector<uint32> string_storage( n_words );
/// typedef PackedStream<uint32*, uint8, 2u, false> packed_stream_type;
/// typedef uint32*                                 offsets_iterator;
/// typedef ConcatenatedStringSet<packed_stream_type, offsets_iterator> packed_string_set;
///
/// packed_stream_type packed_stream( plain_view( string_storage ) );
///
/// // build the string set
/// packed_string_set string_set(
///     n_reads,
///     packed_stream,
///     plain_view( offsets_storage ) );
///
/// // setup the offsets, note we need to place a sentinel
/// for (uint32 i = 0; i < n_reads+1; ++i)
///     offsets_storage[i] = i * read_len;
///
/// // and now we can conveniently access the i-th string of the set
/// for (uint32 i = 0; i < n_reads; ++i)
/// {
///     packed_string_set::string_type string = string_set[i];
///
///     // and fill them in
///     for (uint32 j = 0; j < string.length(); ++j)
///         string[j] = ...
/// }
///\endcode
///
template <typename StringIterator, typename OffsetIterator>
struct ConcatenatedStringSet
{
    typedef concatenated_string_set_tag                                 string_set_tag;
    typedef typename std::iterator_traits<StringIterator>::value_type   symbol_type;
    typedef vector_view<StringIterator>                                 string_type;
    typedef StringIterator                                              symbol_iterator;
    typedef OffsetIterator                                              offset_iterator;
    typedef typename iterator_system<StringIterator>::type              system_tag;

    typedef StringSetIterator< ConcatenatedStringSet<StringIterator,OffsetIterator> >       iterator;
    typedef StringSetIterator< ConcatenatedStringSet<StringIterator,OffsetIterator> > const_iterator;

    /// default constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    ConcatenatedStringSet() {}

    /// constructor
    ///
    /// \param size             set size
    /// \param string           flat string iterator
    /// \param offsets          string offsets in the flat string array, must contain size+1 entries
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    ConcatenatedStringSet(
        const uint32         size,
        const StringIterator string,
        const OffsetIterator offsets) :
        m_size( size ),
        m_string( string ),
        m_offsets( offsets ) {}

    /// set size
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 size() const { return m_size; }

    /// indexing operator: access the i-th string
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    string_type operator[] (const uint32 i) const
    {
        const typename std::iterator_traits<OffsetIterator>::value_type offset = m_offsets[i];

        return string_type(
            m_offsets[i+1] - offset,
            m_string + offset );
    }

    /// begin iterator
    ///
    const_iterator begin() const { return const_iterator(*this,0u); }

    /// begin iterator
    ///
    const_iterator end() const { return const_iterator(*this,size()); }

    /// begin iterator
    ///
    iterator begin() { return iterator(*this,0u); }

    /// begin iterator
    ///
    iterator end() { return iterator(*this,size()); }

    /// return the base string
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    symbol_iterator base_string() const { return m_string; }

    /// return the offset vector
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    offset_iterator offsets() const { return m_offsets; }

private:
    uint32         m_size;
    StringIterator m_string;
    OffsetIterator m_offsets;
};

///\relates ConcatenatedStringSet
///
/// A utility function to make a ConcatenatedStringSet
///
template <typename StringIterator, typename OffsetIterator>
ConcatenatedStringSet<StringIterator,OffsetIterator> make_concatenated_string_set(
    const uint32         size,
    const StringIterator string,
    const OffsetIterator offsets)
{
    return ConcatenatedStringSet<StringIterator,OffsetIterator>(
        size,
        string,
        offsets );
}

///
/// A sparse collection of strings that are stored as ranges of a larger
/// string, and their starting points and end points are given by a range vector.
///
/// \tparam StringIterator      base string support
/// \tparam RangeIterator       an iterator definining a set of ranges, whose value_type
///                             must be <i>uint2</i>.
///
/// Assume you have a large packed-DNA genome and have identified a few isolated regions
/// of importance that you want to analyze. With the following container you can
/// easily represent them:
///
/// \anchor SparseStringSetExample
///\code
/// void analyze_regions(
///     const thrust::device_vector<uint32>& genome_storage,
///     const thrust::device_vector<uint2>&  regions)
/// {
///     typedef PackedStream<const uint32*, uint8, 2u, true>        packed_iterator;
///     typedef const uint2*                                        ranges_iterator;
///     typedef SparseStringSet<packed_iterator, ranges_iterator>   sparse_string_set;
///
///     packed_iterator packed_stream( plain_view( genome_storage ) );
///
///     // build the string set
///     sparse_string_set string_set(
///         regions.size(),
///         packed_stream,
///         plain_view( regions ) );
///
///     // work with the string set
///     ...
/// }
///\endcode
///
template <typename StringIterator, typename RangeIterator>
struct SparseStringSet
{
    typedef sparse_string_set_tag                                       string_set_tag;
    typedef typename std::iterator_traits<StringIterator>::value_type   symbol_type;
    typedef vector_view<StringIterator>                                 string_type;
    typedef StringIterator                                              symbol_iterator;
    typedef RangeIterator                                               range_iterator;
    typedef typename iterator_system<StringIterator>::type              system_tag;

    typedef StringSetIterator< SparseStringSet<StringIterator,RangeIterator> >       iterator;
    typedef StringSetIterator< SparseStringSet<StringIterator,RangeIterator> > const_iterator;

    /// default constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    SparseStringSet() {}

    /// constructor
    ///
    /// \param size             set size
    /// \param string           flat string iterator
    /// \param ranges           string ranges in the flat string array
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    SparseStringSet(
        const uint32         size,
        const StringIterator string,
        const RangeIterator  ranges) :
        m_size( size ),
        m_string( string ),
        m_ranges( ranges ) {}

    /// set size
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 size() const { return m_size; }

    /// indexing operator: access the i-th string
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    string_type operator[] (const uint32 i) const
    {
        const uint2 range = m_ranges[i];

        return string_type(
            range.y - range.x,
            m_string + range.x );
    }

    /// begin iterator
    ///
    const_iterator begin() const { return const_iterator(*this,0u); }

    /// begin iterator
    ///
    const_iterator end() const { return const_iterator(*this,size()); }

    /// begin iterator
    ///
    iterator begin() { return iterator(*this,0u); }

    /// begin iterator
    ///
    iterator end() { return iterator(*this,size()); }

    /// return the base string
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    symbol_iterator base_string() const { return m_string; }

    /// return the offset vector
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    range_iterator ranges() const { return m_ranges; }

private:
    uint32         m_size;
    StringIterator m_string;
    RangeIterator  m_ranges;
};

///\relates SparseStringSet
///
/// A utility function to make a SparseStringSet
///
template <typename StringIterator, typename RangeIterator>
SparseStringSet<StringIterator,RangeIterator> make_sparse_string_set(
    const uint32         size,
    const StringIterator string,
    const RangeIterator offsets)
{
    return SparseStringSet<StringIterator,RangeIterator>(
        size,
        string,
        offsets );
}

///
/// A collection of packed strings stored in strided vectors of words.
/// i.e. if the stride is n, the i-th string s_i is stored in the words w(i + 0), w(i + n), w(i + 2n), ...
///
/// <table>
/// <tr><td><b>s0</b></td>     <td><b>s1</b></td>       <td><b>s2</b></td>      <td>...</td></tr>
/// <tr><td>w(0 + 0)</td>      <td>w(1 + 0)</td>        <td>w(2 + 0)</td>       <td>...</td></tr>
/// <tr><td>w(0 + n)</td>      <td>w(1 + n)</td>        <td>w(2 + n)</td>       <td>...</td></tr>
/// <tr><td>w(0 + 2n)</td>     <td>w(1 + 2n)</td>       <td>w(2 + 2n)</td>      <td>...</td></tr>
/// </table>
///
/// This representation can be convenient for kernels where the i-th thread (modulo the grid-size)
/// operates on the i-th string and the character accesses are in-sync, as in this case all the 
/// memory accesses will be coalesced.
/// \n\n
/// Note that the number of words must be at least <i>n</i> times the number of words needed
/// to store the longest string in the set.
///
template <
    typename StreamIterator,
    typename SymbolType,
    uint32   SYMBOL_SIZE_T,
    bool     BIG_ENDIAN_T,
    typename LengthIterator>
struct StridedPackedStringSet
{
    typedef strided_packed_string_set_tag                                           string_set_tag;
    typedef SymbolType                                                              symbol_type;

    typedef StreamIterator                                                              stream_iterator;
    typedef strided_iterator<StreamIterator>                                            strided_stream_iterator;
    typedef PackedStream<strided_stream_iterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T> packed_stream_type;
    typedef typename packed_stream_type::iterator                                       packed_stream_iterator;
    typedef vector_view<packed_stream_type>                                             string_type;
    typedef LengthIterator                                                              length_iterator;
    typedef typename iterator_system<StreamIterator>::type                              system_tag;

    static const uint32 SYMBOL_SIZE  = SYMBOL_SIZE_T;
    static const bool   BIG_ENDIAN   = BIG_ENDIAN_T;

    /// default constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    StridedPackedStringSet() {}

    /// constructor
    ///
    /// \param size             set size
    /// \param stride           set stride
    /// \param string           flat string iterator
    /// \param lengths          string lengths
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    StridedPackedStringSet(
        const uint32         size,
        const uint32         stride,
        const StreamIterator stream,
        const LengthIterator lengths) :
        m_size( size ),
        m_stride( stride ),
        m_stream( stream ),
        m_lengths( lengths ) {}

    /// set size
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 size() const { return m_size; }

    /// stride
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 stride() const { return m_stride; }

    /// indexing operator: access the i-th string
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    string_type operator[] (const uint32 i) const
    {
        const uint32 length = m_lengths[i];

        const strided_stream_iterator base_iterator( m_stream + i, m_stride );
        const packed_stream_type packed_stream( base_iterator );

        return string_type(
            length,
            packed_stream );
    }

    /// return the base string
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    stream_iterator base_stream() const { return m_stream; }

    /// return the length vector
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    length_iterator lengths() const { return m_lengths; }

private:
    uint32         m_size;
    uint32         m_stride;
    StreamIterator m_stream;
    LengthIterator m_lengths;
};

///
/// A collection of strings stored in strided fashion.
/// i.e. if the stride is n, the i-th string s_i is stored in the symbols s(i + 0), s(i + n), s(i + 2n), ...
///
/// <table>
/// <tr><td><b>s0</b></td>     <td><b>s1</b></td>       <td><b>s2</b></td>      <td>...</td></tr>
/// <tr><td>s(0 + 0)</td>      <td>s(1 + 0)</td>        <td>s(2 + 0)</td>       <td>...</td></tr>
/// <tr><td>s(0 + n)</td>      <td>s(1 + n)</td>        <td>s(2 + n)</td>       <td>...</td></tr>
/// <tr><td>s(0 + 2n)</td>     <td>s(1 + 2n)</td>       <td>s(2 + 2n)</td>      <td>...</td></tr>
/// </table>
///
/// This representation can be convenient for kernels where the i-th thread (modulo the grid-size)
/// operates on the i-th string and the character accesses are in-sync, as in this case all the 
/// memory accesses will be coalesced.
/// \n\n
/// Note that the number of symbols must be at least <i>n</i> times the length of the longest
/// string in the set.
///
template <
    typename StringIterator,
    typename LengthIterator>
struct StridedStringSet
{
    typedef strided_string_set_tag                                                  string_set_tag;
    typedef typename std::iterator_traits<StringIterator>::value_type               symbol_type;

    typedef StringIterator                                                          symbol_iterator;
    typedef strided_iterator<StringIterator>                                        strided_symbol_iterator;
    typedef vector_view<strided_symbol_iterator>                                    string_type;
    typedef LengthIterator                                                          length_iterator;
    typedef typename iterator_system<StringIterator>::type                          system_tag;

    typedef StringSetIterator< StridedStringSet<StringIterator,LengthIterator> >       iterator;
    typedef StringSetIterator< StridedStringSet<StringIterator,LengthIterator> > const_iterator;

    /// default constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    StridedStringSet() {}

    /// constructor
    ///
    /// \param size             set size
    /// \param stride           set stride
    /// \param string           flat string iterator
    /// \param lengths          string lengths
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    StridedStringSet(
        const uint32         size,
        const uint32         stride,
        const StringIterator string,
        const LengthIterator lengths) :
        m_size( size ),
        m_stride( stride ),
        m_string( string ),
        m_lengths( lengths ) {}

    /// set size
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 size() const { return m_size; }

    /// stride
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 stride() const { return m_stride; }

    /// indexing operator: access the i-th string
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    string_type operator[] (const uint32 i) const
    {
        const uint32 length = m_lengths[i];

        const strided_symbol_iterator base_iterator( m_string + i, m_stride );

        return string_type(
            length,
            base_iterator );
    }

    /// begin iterator
    ///
    const_iterator begin() const { return const_iterator(*this,0u); }

    /// begin iterator
    ///
    const_iterator end() const { return const_iterator(*this,size()); }

    /// begin iterator
    ///
    iterator begin() { return iterator(*this,0u); }

    /// begin iterator
    ///
    iterator end() { return iterator(*this,size()); }

    /// return the base string
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    symbol_iterator base_string() const { return m_string; }

    /// return the length vector
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    length_iterator lengths() const { return m_lengths; }

private:
    uint32         m_size;
    uint32         m_stride;
    StringIterator m_string;
    LengthIterator m_lengths;
};

/// A functor fetching the length of the i-th string in a set
///
template <typename string_set_type>
struct string_set_length_functor
{
    typedef uint32 argument_type;
    typedef uint32 result_type;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    string_set_length_functor(const string_set_type _string_set) : string_set(_string_set) {}

    /// return the length of the i-th string, rounded to Q
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 operator() (const uint32 i) const { return string_set[i].length(); }

    const string_set_type string_set;
};

///@} StringSetsModule
///@} Strings

namespace cuda {

///@addtogroup Strings
///@{

///@addtogroup StringSetsModule
///@{

/// copy a generic string set into a concatenated one
///
/// \param in_string_set        input string set
/// \param out_string_set       output string set
///
template <
    typename InStringSet,
    typename StringIterator,
    typename OffsetIterator>
void copy(
    const InStringSet&                                          in_string_set,
          ConcatenatedStringSet<StringIterator,OffsetIterator>& out_string_set);

/// copy a generic string set into a strided one
///
/// \param in_string_set        input string set
/// \param out_string_set       output string set
///
template <
    typename InStringSet,
    typename StringIterator,
    typename LengthIterator>
void copy(
    const InStringSet&                                      in_string_set,
          StridedStringSet<StringIterator,LengthIterator>&  out_string_set);

/// copy a generic string set into a strided-packed one
///
/// \param in_string_set        input string set
/// \param out_string_set       output string set
///
template <
    typename InStringSet,
    typename StreamIterator,
    typename SymbolType,
    uint32   SYMBOL_SIZE_T,
    bool     BIG_ENDIAN_T,
    typename LengthIterator>
void copy(
    const InStringSet&                                                                                  in_string_set,
          StridedPackedStringSet<StreamIterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T,LengthIterator>&  out_string_set);

///@} StringSetsModule
///@} Strings

} // namespace cuda

///@addtogroup Strings
///@{

///@addtogroup StringSetsModule
///@{

/// copy a generic string set into a concatenated one
///
/// \param in_string_set        input string set
/// \param out_string_set       output string set
///
template <
    typename InStringSet,
    typename StringIterator,
    typename OffsetIterator>
void copy(
    const InStringSet&                                          in_string_set,
          ConcatenatedStringSet<StringIterator,OffsetIterator>& out_string_set);

/// copy a generic string set into a strided one
///
/// \param in_string_set        input string set
/// \param out_string_set       output string set
///
template <
    typename InStringSet,
    typename StringIterator,
    typename LengthIterator>
void copy(
    const InStringSet&                                      in_string_set,
          StridedStringSet<StringIterator,LengthIterator>&  out_string_set);

/// copy a generic string set into a strided-packed one
///
/// \param in_string_set        input string set
/// \param out_string_set       output string set
///
template <
    typename InStringSet,
    typename StreamIterator,
    typename SymbolType,
    uint32   SYMBOL_SIZE_T,
    bool     BIG_ENDIAN_T,
    typename LengthIterator>
void copy(
    const InStringSet&                                                                                  in_string_set,
          StridedPackedStringSet<StreamIterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T,LengthIterator>&  out_string_set);

///@} StringSetsModule
///@} Strings


template <
    typename StreamIterator,
    typename SymbolType,
    uint32   SYMBOL_SIZE_T,
    bool     BIG_ENDIAN_T,
    typename LengthIterator,
    typename value_type>
struct CachedPackedConcatStringSet
{
};

template <
    typename StreamIterator,
    typename SymbolType,
    uint32   SYMBOL_SIZE_T,
    bool     BIG_ENDIAN_T,
    typename LengthIterator>
struct CachedPackedConcatStringSet<
    StreamIterator,
    SymbolType,
    SYMBOL_SIZE_T,
    BIG_ENDIAN_T,
    LengthIterator,
    uint4>
{
    typedef const_cached_iterator<StreamIterator>                                       cached_base_iterator;
    typedef uint4_as_uint32_iterator<cached_base_iterator>                              uint4_iterator;
    typedef const_cached_iterator<uint4_iterator>                                       cached_stream_iterator;
    typedef PackedStream<cached_stream_iterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T>  cached_packed_stream_type;
    typedef ConcatenatedStringSet<cached_packed_stream_type,LengthIterator>             cached_string_set;

    static cached_string_set make(
        const ConcatenatedStringSet<
            PackedStream<StreamIterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T>,
            LengthIterator> string_set)
    {
        cached_packed_stream_type cached_packed_stream(
            cached_stream_iterator(
                uint4_iterator( cached_base_iterator( string_set.base_string().stream() ) )
                )
            );

        return cached_string_set(
            string_set.size(),
            cached_packed_stream,
            string_set.offsets() );
    }
};

template <
    typename StreamIterator,
    typename SymbolType,
    uint32   SYMBOL_SIZE_T,
    bool     BIG_ENDIAN_T,
    typename LengthIterator>
struct CachedPackedConcatStringSet<
    StreamIterator,
    SymbolType,
    SYMBOL_SIZE_T,
    BIG_ENDIAN_T,
    LengthIterator,
    uint32>
{
    typedef const_cached_iterator<StreamIterator>                                       cached_stream_iterator;
    typedef PackedStream<cached_stream_iterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T>  cached_packed_stream_type;
    typedef ConcatenatedStringSet<cached_packed_stream_type,LengthIterator>             cached_string_set;

    static cached_string_set make(
        const ConcatenatedStringSet<
            PackedStream<StreamIterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T>,
            LengthIterator> string_set)
    {
        cached_packed_stream_type cached_packed_stream(
            cached_stream_iterator(
                    string_set.base_string().stream() )
            );

        return cached_string_set(
            string_set.size(),
            cached_packed_stream,
            string_set.offsets() );
    }
};

template <
    typename StreamIterator,
    typename SymbolType,
    uint32   SYMBOL_SIZE_T,
    bool     BIG_ENDIAN_T,
    typename LengthIterator,
    typename value_type>
struct CachedPackedSparseStringSet
{
};

template <
    typename StreamIterator,
    typename SymbolType,
    uint32   SYMBOL_SIZE_T,
    bool     BIG_ENDIAN_T,
    typename LengthIterator>
struct CachedPackedSparseStringSet<
    StreamIterator,
    SymbolType,
    SYMBOL_SIZE_T,
    BIG_ENDIAN_T,
    LengthIterator,
    uint4>
{
    typedef const_cached_iterator<StreamIterator>                                       cached_base_iterator;
    typedef uint4_as_uint32_iterator<cached_base_iterator>                              uint4_iterator;
    typedef const_cached_iterator<uint4_iterator>                                       cached_stream_iterator;
    typedef PackedStream<cached_stream_iterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T>  cached_packed_stream_type;
    typedef SparseStringSet<cached_packed_stream_type,LengthIterator>               cached_string_set;

    static cached_string_set make(
        const SparseStringSet<
            PackedStream<StreamIterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T>,
            LengthIterator> string_set)
    {
        cached_packed_stream_type cached_packed_stream(
            cached_stream_iterator(
                uint4_iterator( cached_base_iterator( string_set.base_string().stream() ) )
                )
            );

        return cached_string_set(
            string_set.size(),
            cached_packed_stream,
            string_set.ranges() );
    }
};

template <
    typename StreamIterator,
    typename SymbolType,
    uint32   SYMBOL_SIZE_T,
    bool     BIG_ENDIAN_T,
    typename LengthIterator>
struct CachedPackedSparseStringSet<
    StreamIterator,
    SymbolType,
    SYMBOL_SIZE_T,
    BIG_ENDIAN_T,
    LengthIterator,
    uint32>
{
    typedef const_cached_iterator<StreamIterator>                                       cached_stream_iterator;
    typedef PackedStream<cached_stream_iterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T>  cached_packed_stream_type;
    typedef SparseStringSet<cached_packed_stream_type,LengthIterator>                   cached_string_set;

    static cached_string_set make(
        const SparseStringSet<
            PackedStream<StreamIterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T>,
            LengthIterator> string_set)
    {
        cached_packed_stream_type cached_packed_stream(
            cached_stream_iterator(
                    string_set.base_string().stream() )
            );

        return cached_string_set(
            string_set.size(),
            cached_packed_stream,
            string_set.ranges() );
    }
};

///
/// A utility function to convert a plain packed-sparse string set into a cached one
///
template <
    typename StreamIterator,
    typename SymbolType,
    uint32   SYMBOL_SIZE_T,
    bool     BIG_ENDIAN_T,
    typename LengthIterator>
typename CachedPackedSparseStringSet<
    StreamIterator,
    SymbolType,
    SYMBOL_SIZE_T,
    BIG_ENDIAN_T,
    LengthIterator,
    typename std::iterator_traits<StreamIterator>::value_type>::cached_string_set
make_cached_string_set(
    const SparseStringSet<
        PackedStream<StreamIterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T>,
        LengthIterator> string_set)
{
    typedef CachedPackedSparseStringSet<
        StreamIterator,
        SymbolType,
        SYMBOL_SIZE_T,
        BIG_ENDIAN_T,
        LengthIterator,
        typename std::iterator_traits<StreamIterator>::value_type> Adapter;

    return Adapter::make( string_set );
}

///
/// A utility function to convert a plain packed-concatenated string set into a cached one
///
template <
    typename StreamIterator,
    typename SymbolType,
    uint32   SYMBOL_SIZE_T,
    bool     BIG_ENDIAN_T,
    typename LengthIterator>
typename CachedPackedConcatStringSet<
    StreamIterator,
    SymbolType,
    SYMBOL_SIZE_T,
    BIG_ENDIAN_T,
    LengthIterator,
    typename std::iterator_traits<StreamIterator>::value_type>::cached_string_set
make_cached_string_set(
    const ConcatenatedStringSet<
        PackedStream<StreamIterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T>,
        LengthIterator> string_set)
{
    typedef CachedPackedConcatStringSet<
        StreamIterator,
        SymbolType,
        SYMBOL_SIZE_T,
        BIG_ENDIAN_T,
        LengthIterator,
        typename std::iterator_traits<StreamIterator>::value_type> Adapter;

    return Adapter::make( string_set );
}

} // namespace nvbio

#include <nvbio/strings/string_set_inl.h>
