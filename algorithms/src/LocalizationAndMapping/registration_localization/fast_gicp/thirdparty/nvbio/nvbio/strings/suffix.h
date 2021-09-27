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

#include <nvbio/strings/string_set.h>


namespace nvbio {

///@addtogroup Strings
///@{

///@addtogroup StringSetsModule
///@{

/// 32-bit string suffix coordinates type
///
typedef uint32 string_suffix_coord_type;

/// 64-bit string suffix coordinates type
///
typedef uint64 long_string_suffix_coord_type;

/// 32-bit string-set suffix coordinates type
///
typedef uint32_2 string_set_suffix_coord_type;

/// 64-bit string-set suffix coordinates type
///
typedef uint64_2 long_string_set_suffix_coord_type;

///@addtogroup Private
///@{

/// A class to represent a string suffix, i.e. an arbitrarily placed substring
///
/// \tparam StringType          the underlying string type
/// \tparam CoordType           the type of suffix coordinates, string_suffix_coord_type for strings, string_set_suffix_coord_type for string-sets
/// \tparam CoordDim            the number of coordinates, 1 for strings, 2 for string-sets
///
template <
    typename StringType,
    typename CoordType,
    uint32   CoordDim>
struct SuffixCore {};

/// A class to represent a string suffix, i.e. an arbitrarily placed substring
///
/// \tparam StringType          the underlying string type
/// \tparam CoordType           the type of suffix coordinates, uint32|uint64
///
template <
    typename StringType,
    typename CoordType>
struct SuffixCore<StringType,CoordType,1u>
{
    typedef StringType                                              string_type;
    typedef CoordType                                               coord_type;
    typedef typename vector_traits<CoordType>::value_type           index_type;

    typedef typename std::iterator_traits<string_type>::value_type  symbol_type;
    typedef typename std::iterator_traits<string_type>::value_type  value_type;
    typedef typename std::iterator_traits<string_type>::reference   reference;

    typedef typename string_traits<StringType>::iterator            iterator;
    typedef typename string_traits<StringType>::const_iterator      const_iterator;
    typedef typename string_traits<StringType>::forward_iterator    forward_iterator;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    SuffixCore() {}

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    SuffixCore(
        const string_type   string,
        const coord_type    suffix) :
        m_string( string ),
        m_coords( suffix ) {}

    /// suffix size
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 size() const { return nvbio::length( m_string ) - m_coords; }

    /// suffix length
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 length() const { return size(); }

    /// indexing operator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    symbol_type operator[] (const uint32 i) const { return m_string[ m_coords + i ]; }

    /// indexing operator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    reference operator[] (const uint32 i) { return m_string[ m_coords + i ]; }

    /// return the suffix coordinates
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    coord_type coords() const { return m_coords; }

    /// return an iterator
    ///
    iterator begin() { return m_string.begin() + m_coords; }

    /// return an iterator
    ///
    iterator end() { return m_string.end(); }

    /// return an iterator
    ///
    const_iterator begin() const { return m_string.begin() + m_coords; }

    /// return an iterator
    ///
    const_iterator end() const { return m_string.end(); }

    string_type     m_string;       ///< the underlying string set
    coord_type      m_coords;       ///< the suffix coordinates
};

/// A class to represent a string suffix, i.e. an arbitrarily placed substring
///
/// \tparam StringType          the underlying string type
/// \tparam CoordType           the type of suffix coordinates, uint32|uint64
///
template <
    typename StringType,
    typename CoordType>
struct SuffixCore<StringType,CoordType,2u>
{
    typedef StringType                                              string_type;
    typedef CoordType                                               coord_type;
    typedef typename vector_traits<CoordType>::value_type           index_type;

    typedef typename std::iterator_traits<string_type>::value_type  symbol_type;
    typedef typename std::iterator_traits<string_type>::value_type  value_type;
    typedef typename std::iterator_traits<string_type>::reference   reference;

    typedef typename string_traits<StringType>::iterator            iterator;
    typedef typename string_traits<StringType>::const_iterator      const_iterator;
    typedef typename string_traits<StringType>::forward_iterator    forward_iterator;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    SuffixCore() {}

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    SuffixCore(
        const string_type   string,
        const coord_type    suffix) :
        m_string( string ),
        m_coords( suffix ) {}

    /// suffix size
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 size() const { return nvbio::length( m_string ) - m_coords.y; }

    /// suffix length
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 length() const { return size(); }

    /// indexing operator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    symbol_type operator[] (const uint32 i) const { return m_string[ m_coords.y + i ]; }

    /// indexing operator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    reference operator[] (const uint32 i) { return m_string[ m_coords.y + i ]; }

    /// return the suffix coordinates
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    coord_type coords() const { return m_coords; }

    /// return an iterator
    ///
    iterator begin() { return m_string.begin() + m_coords.y; }

    /// return an iterator
    ///
    iterator end() { return m_string.end(); }

    /// return an iterator
    ///
    const_iterator begin() const { return m_string.begin() + m_coords.y; }

    /// return an iterator
    ///
    const_iterator end() const { return m_string.end(); }

    string_type     m_string;       ///< the underlying string set
    coord_type      m_coords;       ///< the suffix coordinates
};


///@} Private

/// A class to represent a string suffix, i.e. an arbitrarily placed substring
///
/// \tparam StringType          the underlying string type
/// \tparam CoordType           the type of suffix coordinates, string_suffix_coord_type for strings, string_set_suffix_coord_type for string-sets
/// \tparam CoordDim            the number of coordinates, 1 for strings, 2 for string-sets
///
template <
    typename StringType,
    typename CoordType>
struct Suffix : SuffixCore< StringType, CoordType, vector_traits<CoordType>::DIM >
{
    static const uint32 SYMBOL_SIZE = string_traits<StringType>::SYMBOL_SIZE;

    typedef SuffixCore< StringType, CoordType, vector_traits<CoordType>::DIM >  core_type;
    typedef StringType                                                          string_type;
    typedef CoordType                                                           coord_type;
    typedef typename vector_traits<CoordType>::value_type                       index_type;

    typedef typename std::iterator_traits<string_type>::value_type              symbol_type;
    typedef typename std::iterator_traits<string_type>::value_type              value_type;
    typedef typename std::iterator_traits<string_type>::reference               reference;

    typedef typename string_traits<StringType>::iterator                        iterator;
    typedef typename string_traits<StringType>::const_iterator                  const_iterator;
    typedef typename string_traits<StringType>::forward_iterator                forward_iterator;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    Suffix() {}

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    Suffix(
        const string_type   string,
        const coord_type    infix) : core_type( string, infix ) {}
};

/// make a suffix, i.e. a substring of a given string
///
/// \tparam StringType  the underlying string type
/// \tparam CoordType   the coordinates type, either string_suffix_coord_type or string_set_suffix_coord_type
///
/// \param string       the underlying string object
/// \param coords       the suffix coordinates
///
template <typename StringType, typename CoordType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
Suffix<StringType,CoordType> make_suffix(const StringType string, const CoordType coords)
{
    return Suffix<StringType,CoordType>( string, coords );
}

///@addtogroup Private
///@{

/// Represent a set of suffixes of a string or string-set
///
/// \tparam SequenceType            the string or string-set type
/// \tparam SuffixIterator          the suffix iterator type - value_type can be string_suffix_coord_type for strings, string_set_suffix_coord_type for string-sets
/// \tparam CoordDim                the number of coordinates representing a suffix, 1 for strings, 2 for string-sets
///
template <
    typename SequenceType,
    typename SuffixIterator,
    uint32   CoordDim>
struct SuffixSetCore {};

/// Represent a set of suffixes of a string
///
/// \tparam SequenceType            the string or string-set container
/// \tparam SuffixIterator          the suffix iterator type - value_type can be uint32 or uint64
///
template <
    typename SequenceType,
    typename SuffixIterator>
struct SuffixSetCore<SequenceType,SuffixIterator,1u>
{
    typedef SequenceType                                                sequence_type;
    typedef SuffixIterator                                              suffix_iterator;

    typedef typename std::iterator_traits<SuffixIterator>::value_type   coord_type;
    typedef Suffix<sequence_type, coord_type>                           string_type;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    SuffixSetCore() {}

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    SuffixSetCore(
        const uint32            size,
        const sequence_type     sequence,
        const suffix_iterator    suffixes) :
        m_size( size ),
        m_sequence( sequence ),
        m_suffixes( suffixes ) {}

    /// set size
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 size() const { return m_size; }

    /// indexing operator: access the i-th string
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    string_type operator[] (const uint32 i) const
    {
        const coord_type coords = m_suffixes[i];
        return string_type( m_sequence, coords );
    }

    uint32              m_size;
    sequence_type       m_sequence;
    suffix_iterator     m_suffixes;
};

/// Represent a set of suffixes of a string-set
///
/// \tparam SequenceType            the string or string-set type
/// \tparam SuffixIterator          the suffix iterator type - value_type can be string_set_suffix_coord_type or long_string_set_suffix_coord_type
///
template <
    typename SequenceType,
    typename SuffixIterator>
struct SuffixSetCore<SequenceType,SuffixIterator,2u>
{
    typedef SequenceType                                                sequence_type;
    typedef SuffixIterator                                              suffix_iterator;

    typedef typename sequence_type::string_type                         base_string_type;
    typedef typename std::iterator_traits<SuffixIterator>::value_type   coord_type;
    typedef Suffix<base_string_type, coord_type>                        string_type;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    SuffixSetCore() {}

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    SuffixSetCore(
        const uint32            size,
        const sequence_type     sequence,
        const suffix_iterator   suffixes) :
        m_size( size ),
        m_sequence( sequence ),
        m_suffixes( suffixes ) {}

    /// set size
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 size() const { return m_size; }

    /// indexing operator: access the i-th string
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    string_type operator[] (const uint32 i) const
    {
        const coord_type coords = m_suffixes[i];
        return string_type( m_sequence[ coords.x ], coords );
    }

    uint32              m_size;
    sequence_type       m_sequence;
    suffix_iterator     m_suffixes;
};

///@} Private

/// return the string index of a given suffix
///
template <typename StringType, typename CoordType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32 string_id(const SuffixCore<StringType,CoordType,2u>& suffix) { return suffix.m_coords.x; }

/// return the length of a given suffix
///
template <typename StringType, typename CoordType, uint32 CoordDim>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32 length(const SuffixCore<StringType,CoordType,CoordDim>& suffix) { return suffix.length(); }

///\anchor SuffixSets
///\par
/// Represent a set of suffixes of a string or string-set. A SuffixSet is a \ref StringSetAnchor "String Set".
///\par
/// For a string T[0,...,n-1], a suffix is a substring T[i,n). A SuffixSet is hence defined
/// by a string and a collection of indices { i_0, ..., i_m }.
/// These coordinates must be of type \ref string_suffix_coord_type or \ref long_string_suffix_coord_type.
///\par
/// For a string-set, an prefix is instead defined as a pair (k,i), where k denotes the string index in
/// the set and i denotes the suffix starting coordinate.
/// These coordinates must be of type \ref string_set_suffix_coord_type or \ref long_string_set_suffix_coord_type.
///
/// \tparam SequenceType        the string or string-set type
/// \tparam SuffixIterator      the suffix iterator type - value_type can be string_suffix_coord_type for strings, string_set_suffix_coord_type
///
template <
    typename SequenceType,
    typename SuffixIterator>
struct SuffixSet : public SuffixSetCore<
                            SequenceType,
                            SuffixIterator,
                            vector_traits<typename std::iterator_traits<SuffixIterator>::value_type>::DIM>
{
    typedef SuffixSetCore<
        SequenceType,
        SuffixIterator,
        vector_traits<typename std::iterator_traits<SuffixIterator>::value_type>::DIM>   base_type;

    typedef SequenceType                                                sequence_type;      ///< the underlying sequence type
    typedef SuffixIterator                                              suffix_iterator;    ///< the underlingy suffix iterator type
    typedef typename iterator_system<SuffixIterator>::type              system_tag;         ///< the system tag

    typedef typename base_type::coord_type                              coord_type;         ///< the suffix coordinates type
    typedef typename base_type::string_type                             string_type;        ///< the suffix string type

    typedef StringSetIterator< SuffixSet<SequenceType,SuffixIterator> >        iterator;     ///< the iterator type
    typedef StringSetIterator< SuffixSet<SequenceType,SuffixIterator> >  const_iterator;     ///< the const_iterator type

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    SuffixSet() {}

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    SuffixSet(
        const uint32            size,
        const sequence_type     sequence,
        const suffix_iterator    suffixes) :
        base_type( size, sequence, suffixes ) {}

    /// begin iterator
    ///
    const_iterator begin() const { return const_iterator(*this,0u); }

    /// begin iterator
    ///
    const_iterator end() const { return const_iterator(*this,base_type::size()); }

    /// begin iterator
    ///
    iterator begin() { return iterator(*this,0u); }

    /// begin iterator
    ///
    iterator end() { return iterator(*this,base_type::size()); }
};

/// compare string-set suffixes
///
template <typename string_set_type>
int32 compare_suffixes(
    const string_set_type string_set,
    const uint2           suffix1,
    const uint2           suffix2)
{
    typedef typename string_set_type::string_type   string_type;
    typedef Suffix<string_type,uint32>              suffix_type;

    // check whether both inputs represent the same suffix
    if (suffix1.y == suffix2.y &&
        suffix1.x == suffix2.x)
        return 0;

    const suffix_type string1 = make_suffix( string_set[suffix1.y], suffix1.x );
    const suffix_type string2 = make_suffix( string_set[suffix2.y], suffix2.x );

    const uint32 len1 = nvbio::length( string1 );
    const uint32 len2 = nvbio::length( string2 );

    const uint32 min_len = nvbio::min( len1, len2 );

    // compare character by character
    int32 cmp = 0;
    for (uint32 j = 0; j < min_len; ++j)
    {
        const uint8 c_i = string1[j];
        const uint8 c_n = string2[j];
        if (c_i < c_n)
        {
            cmp = -1;
            break;
        }
        if (c_i > c_n)
        {
            cmp = 1;
            break;
        }
    }

    // break ties...
    if (cmp == 0)
    {
        if (len1 < len2)      // $ is smaller than any other character => s1 < s2
            cmp = -1; 
        else if (len2 < len1) // $ is smaller than any other character => s2 < s1
            cmp =  1;
        else                  // $_1 < $_2 ? -1 : 1
            cmp = suffix1.y < suffix2.y ? -1 : 1;
    }
    return cmp;
}

///@} StringSetsModule
///@} Strings

} // namespace nvbio

//#include <nvbio/basic/suffix_inl.h>
