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

/*! \file cached_iterator.h
 *   \brief CUDA-compatible iterator wrappers allowing to cache the dereferenced
 *   value of generic iterators
 */

#pragma once

#include <nvbio/basic/types.h>
#include <nvbio/basic/iterator.h>

namespace nvbio {

///@addtogroup Basic
///@{

///@addtogroup Iterators
///@{

///
/// A simple class to cache accesses to a stream into a register. For example,
/// this iterator can be useful to wrap the underlying (uint32 or uint4) storage
/// of PackedStream's, so as to avoid repeated accesses to long latency memories.
///
/// \section CachedIteratorExample Example
///
/// \code
/// void write_stream(uint32* gmem_storage)
/// {
///     typedef nvbio::cached_iterator<const uint32*> cached_storage_type;
///     nvbio::PackedStream<cached_storage_type, uint8, 2, false> packed_stream( gmem_storage );
///     packed_stream[0]  = 1;
///     packed_stream[5]  = 0;
///     packed_stream[10] = 1;
///     packed_stream[15] = 0;      // all the writes above, which happen to hit the same word
///                                 // gmem_storage[0], will be cached.
///     packed_stream[20] = 1;      // this write will trigger a new memory access to gmem_storage[1].
///     return packed_stream[25];   // this read will be cached.
/// }
/// \endcode
///
template <typename InputStream>
struct cached_iterator
{
    typedef typename std::iterator_traits<InputStream>::value_type      value_type;
    typedef typename std::iterator_traits<InputStream>::reference       reference;
    typedef typename std::iterator_traits<InputStream>::pointer         pointer;
    typedef typename std::iterator_traits<InputStream>::difference_type difference_type;
    //typedef typename std::iterator_traits<InputStream>::distance_type   distance_type;
    typedef typename std::random_access_iterator_tag                    iterator_category;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE cached_iterator()
        : m_cache_idx(uint32(-1)) {}

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE cached_iterator(InputStream stream)
        : m_stream( stream ), m_cache_idx(uint32(-1)), m_cache_val(value_type()) {}

    /// destructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE ~cached_iterator()
    {
        if (m_cache_idx != uint32(-1))
            m_stream[ m_cache_idx ] = m_cache_val;
    }

    /// indexing operator
    ///
    /// \param i        requested value
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE value_type& operator[] (const uint32 i)
    {
        if (m_cache_idx == i)
            return m_cache_val;

        if (m_cache_idx != uint32(-1))
            m_stream[ m_cache_idx ] = m_cache_val;

        m_cache_idx = i;
        m_cache_val = m_stream[i];
        return m_cache_val;
    }

private:
    InputStream m_stream;
    uint32      m_cache_idx;
    value_type  m_cache_val;
};

///
/// A simple class to cache accesses to a stream into a register. For example,
/// this iterator can be useful to wrap the underlying (uint32 or uint4) storage
/// of PackedStream's, so as to avoid repeated accesses to long latency memories.
///
/// \section ConstCachedIteratorExample Example
///
/// \code
/// uint8 read_stream(const uint32* gmem_storage)
/// {
///     typedef nvbio::const_cached_iterator<const uint32*> cached_storage_type;
///     nvbio::PackedStream<cached_storage_type, uint8, 2, false> packed_stream( gmem_storage );
///     return packed_stream[0]  |
///            packed_stream[5]  |
///            packed_stream[10] |
///            packed_stream[15] | // all the reads above, which happen to hit the same word
///                                // gmem_storage[0], will be cached.
///            packed_stream[20];  // this read will trigger a new memory access to gmem_storage[1].
/// }
/// \endcode
///
template <typename InputStream>
struct const_cached_iterator
{
    typedef typename std::iterator_traits<InputStream>::value_type      value_type;
    typedef typename std::iterator_traits<InputStream>::reference       reference;
    typedef typename std::iterator_traits<InputStream>::pointer         pointer;
    typedef typename std::iterator_traits<InputStream>::difference_type difference_type;
    //typedef typename std::iterator_traits<InputStream>::distance_type   distance_type;
    typedef typename std::random_access_iterator_tag                    iterator_category;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE const_cached_iterator() {}

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE const_cached_iterator(InputStream stream)
        : m_stream( stream ), m_cache_idx(uint32(-1)), m_cache_val(value_type()) {}

    /// indexing operator
    ///
    /// \param i        requested value
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE value_type operator[] (const uint32 i) const
    {
        if (m_cache_idx == i)
            return m_cache_val;

        m_cache_idx = i;
        m_cache_val = m_stream[i];
        return m_cache_val;
    }

    /// base stream
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    InputStream stream() const { return m_stream; }

            InputStream m_stream;
    mutable uint32      m_cache_idx;
    mutable value_type  m_cache_val;
};

/// make a cached iterator
///
template <typename InputStream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
cached_iterator<InputStream> make_cached_iterator(InputStream it)
{
    return cached_iterator<InputStream>( it );
}

/// make a const cached iterator
///
template <typename InputStream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
const_cached_iterator<InputStream> make_const_cached_iterator(InputStream it)
{
    return cached_iterator<InputStream>( it );
}

/// less than
///
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool operator< (
    const const_cached_iterator<Stream>& it1,
    const const_cached_iterator<Stream>& it2);

/// greater than
///
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool operator> (
    const const_cached_iterator<Stream>& it1,
    const const_cached_iterator<Stream>& it2);

/// less than
///
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool operator<= (
    const const_cached_iterator<Stream>& it1,
    const const_cached_iterator<Stream>& it2);

/// greater than
///
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool operator>= (
    const const_cached_iterator<Stream>& it1,
    const const_cached_iterator<Stream>& it2);

/// equality test
///
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool operator== (
    const const_cached_iterator<Stream>& it1,
    const const_cached_iterator<Stream>& it2);

/// inequality test
///
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool operator!= (
    const const_cached_iterator<Stream>& it1,
    const const_cached_iterator<Stream>& it2);

/// pre-increment operator
///
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
const_cached_iterator<Stream>& operator++ (const_cached_iterator<Stream>& it);

/// post-increment operator
///
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
const_cached_iterator<Stream> operator++ (const_cached_iterator<Stream>& it, int dummy);

/// pre-decrement operator
///
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
const_cached_iterator<Stream>& operator-- (const_cached_iterator<Stream>& it);

/// post-decrement operator
///
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
const_cached_iterator<Stream> operator-- (const_cached_iterator<Stream>& it, int dummy);

/// add offset
///
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
const_cached_iterator<Stream>& operator+= (const_cached_iterator<Stream>& it, const typename const_cached_iterator<Stream>::difference_type distance);

/// subtract offset
///
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
const_cached_iterator<Stream>& operator-= (const_cached_iterator<Stream>& it, const typename const_cached_iterator<Stream>::difference_type distance);

/// add offset
///
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
const_cached_iterator<Stream> operator+ (const const_cached_iterator<Stream> it, const typename const_cached_iterator<Stream>::difference_type distance);

/// subtract offset
///
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
const_cached_iterator<Stream> operator- (const const_cached_iterator<Stream> it, const typename const_cached_iterator<Stream>::difference_type distance);

/// difference
///
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
typename const_cached_iterator<Stream>::difference_type operator- (const const_cached_iterator<Stream> it1, const const_cached_iterator<Stream> it2);

///@} Iterators
///@} Basic

} // namespace nvbio

#include <nvbio/basic/cached_iterator_inl.h>
