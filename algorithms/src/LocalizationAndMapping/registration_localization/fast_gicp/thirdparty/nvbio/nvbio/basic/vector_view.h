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

/*! \vector_view.h
 *   \brief Define a vector_view POD type and plain_view() for std::vector.
 */

#pragma once

#include <nvbio/basic/types.h>
#include <nvbio/basic/iterator.h>
#include <nvbio/strings/string_traits.h>
#include <iterator>
#include <vector>

namespace nvbio {

/// \page vector_views_page Vector Views
///
/// This module implements a vector adaptor, which allows to create an "std::vector"-like
/// container on top of a base iterator.
///
/// - vector_view
///
/// \section VectorViewExampleSection Example
///
///\code
/// // build a vector_view out of a static array
/// typedef vector_view<uint32*> vector_type;
///
/// uint32 storage[16];
///
/// vector_type vector( 0, storage );
///
/// // use push_back()
/// vector.push_back( 3 );
/// vector.push_back( 7 );
/// vector.push_back( 11 );
///
/// // use resize()
/// vector.resize( 4 );
///
/// // use the indexing operator[]
/// vector[3] = 8;
///
/// // use the begin() / end() iterators
/// std::sort( vector.begin(), vector.end() );
///
/// // use front() and back()
/// printf("(%u, %u)\n");                       // -> (3,11)
///\endcode
///

///@addtogroup Basic
///@{

///
/// Wrapper class to create a "vector"-like container on top of a generic base iterator.
/// See \ref VectorWrapperExampleSection.
///
/// \tparam Iterator        base iterator type
///
template <typename Iterator, typename IndexType = uint32>
struct vector_view
{
    typedef Iterator                                                    iterator;
    typedef Iterator                                                    const_iterator;
    typedef typename iterator_traits<Iterator>::forward_iterator        forward_iterator;

    typedef typename iterator_traits<Iterator>::value_type              value_type;
    typedef typename iterator_traits<Iterator>::reference               reference;
    typedef typename to_const<reference>::type                          const_reference;
    typedef typename iterator_traits<Iterator>::pointer                 pointer;
    typedef IndexType                                                   size_type;
    typedef IndexType                                                   index_type;
    typedef typename iterator_traits<Iterator>::difference_type         difference_type;
    typedef std::random_access_iterator_tag                             iterator_category;

    typedef vector_view<Iterator,IndexType>                                   plain_view_type;
    typedef vector_view<Iterator,IndexType>                             const_plain_view_type;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    vector_view() {}

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    vector_view(const IndexType size, Iterator vec) : m_size( size ), m_vec( vec ) {}

    /// resize the vector
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void resize(const uint32 sz) { m_size = sz; }

    /// clear
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void clear() { m_size = 0; }

    /// return vector size
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    IndexType size() const { return m_size; }

    /// return vector size
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    IndexType length() const { return m_size; }

    /// return true iff size is null
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    bool empty() const { return m_size == 0; }

    /// dereference operator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    const_reference operator*() const                   { NVBIO_CUDA_DEBUG_ASSERT( 0 < m_size, "vector_view: access out of bounds, dereferenced zero sized vector\n" ); return *m_vec; }

    /// dereference operator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    reference operator*()                               { NVBIO_CUDA_DEBUG_ASSERT( 0 < m_size, "vector_view: access out of bounds, dereferenced zero sized vector\n" ); return *m_vec; }

    /// const indexing operator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    const_reference operator[](const IndexType i) const { NVBIO_CUDA_DEBUG_ASSERT( i < m_size, "vector_view: access out of bounds, %llu >= %llu\n", uint64(i), uint64(m_size) ); return m_vec[i]; }

    /// indexing operator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    reference operator[](const IndexType i)             { NVBIO_CUDA_DEBUG_ASSERT( i < m_size, "vector_view: access out of bounds, %llu >= %llu\n", uint64(i), uint64(m_size) ); return m_vec[i]; }

    /// return reference to front
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    const_reference front(void) const { return m_vec[0]; }

    /// return reference to front
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    reference front(void) { return m_vec[0]; }

    /// return reference to back
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    const_reference back(void) const { return m_vec[m_size-1]; }

    /// return reference to back
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    reference back(void) { return m_vec[m_size-1]; }

    /// return the base iterator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    Iterator base() const { return m_vec; }

    /// return begin iterator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    const_iterator begin() const { return m_vec; }

    /// return end iterator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    const_iterator end() const { return m_vec + m_size; }

    /// return begin iterator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    iterator begin() { return m_vec; }

    /// return end iterator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    iterator end() { return m_vec + m_size; }

    /// automatic conversion to the basic iterator type
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    operator Iterator() const { return m_vec; }

    /// push back
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void push_back(const_reference val) { m_vec[ m_size ] = val; m_size++; }

    /// pop back
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void pop_back() { --m_size; }

    IndexType   m_size;
    Iterator    m_vec;
};

///\relates vector_view
/// string_traits specialization for vector_view
///
template <typename Iterator, typename IndexType>
struct string_traits< vector_view<Iterator,IndexType> >
{
    static const uint32 SYMBOL_SIZE  = string_traits<Iterator>::SYMBOL_SIZE;

    typedef typename vector_view<Iterator,IndexType>::value_type        value_type;
    typedef typename vector_view<Iterator,IndexType>::reference         reference;
    typedef typename vector_view<Iterator,IndexType>::index_type        index_type;
    typedef typename vector_view<Iterator,IndexType>::iterator          iterator;
    typedef typename vector_view<Iterator,IndexType>::const_iterator    const_iterator;
    typedef typename vector_view<Iterator,IndexType>::forward_iterator  forward_iterator;
};

/// return length of a vector_view
///
template <typename Iterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32 length(const vector_view<Iterator>& vec) { return vec.length(); }

/// return the raw pointer of a vector_view
///
template <typename T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
T raw_pointer(const vector_view<T>& vec) { return vec.base(); }

/// return the begin iterator of a vector_view
///
template <typename T, typename I>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
T begin(vector_view<T,I>& vec) { return vec.begin(); }

/// return the begin iterator of a vector_view
///
template <typename T, typename I>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
T begin(const vector_view<T,I>& vec) { return vec.begin(); }

//
// --- std::vector views ----------------------------------------------------------------------------
//

/// define the plain view of a std::vector
///
template <typename T> struct plain_view_subtype< std::vector<T> > { typedef vector_view<T*,uint64> type; };

/// define the plain view of a std::vector
///
template <typename T> struct plain_view_subtype< const std::vector<T> > { typedef vector_view<const T*,uint64> type; };


/// return the plain view of a std::vector
///
template <typename T>
vector_view<T*,uint64> plain_view(std::vector<T>& vec) { return vector_view<T*,uint64>( vec.size(), vec.size() ? &vec[0] : NULL ); }

/// return the plain view of a std::vector
///
template <typename T>
vector_view<const T*,uint64> plain_view(const std::vector<T>& vec) { return vector_view<const T*,uint64>( vec.size(), vec.size() ? &vec[0] : NULL ); }

/// return the raw pointer of a std::vector
///
template <typename T>
T* raw_pointer(std::vector<T>& vec) { return vec.size() ? &vec[0] : NULL; }

/// return the raw pointer of a std::vector
///
template <typename T>
const T* raw_pointer(const std::vector<T>& vec) { return vec.size() ? &vec[0] : NULL; }

/// return the begin iterator of a raw pointer
///
template <typename T> T* begin(T* vec) { return vec; }

/// return the begin iterator of a raw pointer
///
template <typename T> const T* begin(const T* vec) { return vec; }

///@} Basic

} // namespace nvbio
