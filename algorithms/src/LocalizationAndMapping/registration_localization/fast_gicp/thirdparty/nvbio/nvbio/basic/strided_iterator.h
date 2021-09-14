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
#include <nvbio/basic/iterator.h>

namespace nvbio {

/// \page iterators_page Iterators
///
/// NVBIO provides a few adaptable iterator classes which can be used to construct
/// different views on top of some underlying iterator:
///
/// - strided_iterator
/// - block_strided_iterator
/// - transform_iterator
/// - index_transform_iterator
/// - cached_iterator
/// - const_cached_iterator
///

///@addtogroup Basic
///@{

///@addtogroup Iterators
///@{

///
/// Wrapper class to create a strided iterator out of another base iterator, i.e:
///
///   it[  <b>j</b>  ] = base[ <b>j</b>  *  <i>stride</i> ]
///
template <typename T>
struct strided_iterator
{
    typedef typename std::iterator_traits<T>::value_type        value_type;
    typedef typename std::iterator_traits<T>::reference         reference;
    typedef typename to_const<reference>::type                  const_reference;
    typedef typename std::iterator_traits<T>::pointer           pointer;
    typedef typename std::iterator_traits<T>::difference_type   difference_type;
    //typedef typename std::iterator_traits<T>::distance_type     distance_type;
    typedef typename std::iterator_traits<T>::iterator_category iterator_category;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    strided_iterator() {}

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    strided_iterator(T vec, const uint32 stride) : m_vec( vec ), m_stride( stride ) {}

    /// const dereferencing operator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    const_reference operator*() const { return *m_vec; }

    /// dereferencing operator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    reference operator*() { return *m_vec; }

    /// const indexing operator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    const_reference operator[](const uint32 i) const { return m_vec[i*m_stride]; }

    /// indexing operator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    reference operator[](const uint32 i) { return m_vec[i*m_stride]; }

    /// addition
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    strided_iterator<T> operator+(const uint32 i) const
    {
        return strided_iterator( m_vec + i * m_stride, m_stride );
    }

    /// iterator subtraction
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    difference_type operator-(const strided_iterator<T> it) const
    {
        return m_vec - it.m_vec;
    }

    /// pre-increment
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    strided_iterator<T>& operator++()
    {
        m_vec += m_stride;
        return *this;
    }

    T      m_vec;
    uint32 m_stride;
};

/// build a strided iterator
///
template <typename T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
strided_iterator<T> make_strided_iterator(T it, const uint32 stride)
{
    return strided_iterator<T>( it, stride );
}

/// operator ==
///
template <typename T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool operator==(const strided_iterator<T> it1, const strided_iterator<T> it2)
{
    return (it1.m_vec == it2.m_vec) && (it1.m_stride == it2.m_stride);
}
/// operator !=
///
template <typename T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool operator!=(const strided_iterator<T> it1, const strided_iterator<T> it2)
{
    return (it1.m_vec != it2.m_vec) || (it1.m_stride != it2.m_stride);
}
/// operator <
///
template <typename T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool operator<(const strided_iterator<T> it1, const strided_iterator<T> it2) { return (it1.m_vec < it2.m_vec); }
/// operator <=
///
template <typename T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool operator<=(const strided_iterator<T> it1, const strided_iterator<T> it2) { return (it1.m_vec <= it2.m_vec); }
/// operator >
///
template <typename T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool operator>(const strided_iterator<T> it1, const strided_iterator<T> it2) { return (it1.m_vec > it2.m_vec); }
/// operator >=
///
template <typename T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool operator>=(const strided_iterator<T> it1, const strided_iterator<T> it2) { return (it1.m_vec >= it2.m_vec); }

enum block_strided_layout
{
    ROW_MAJOR_LAYOUT    = 0u,
    COLUMN_MAJOR_LAYOUT = 1u
};

///
/// Wrapper class to create a block-strided iterator out of another base iterator, i.e:
///
///   it[  <b>j</b>  ] = base[ (<b>j</b>  /  BLOCKSIZE) * <i>stride</i> + (<b>j</b>  %%  BLOCKSIZE) ]
///
/// or:
///
///   it[  <b>j</b>  ] = base[ (<b>j</b>  %%  BLOCKSIZE) * <i>stride</i> + (<b>j</b> / BLOCKSIZE) ]
///
/// depending on whether the alignment is ROW_MAJOR or COLUMN_MAJOR
///
template <uint32 BLOCKSIZE, typename T, block_strided_layout LAYOUT = ROW_MAJOR_LAYOUT>
struct block_strided_iterator
{
    typedef typename std::iterator_traits<T>::value_type        value_type;
    typedef typename std::iterator_traits<T>::reference         reference;
    typedef typename to_const<reference>::type                  const_reference;
    typedef typename std::iterator_traits<T>::pointer           pointer;
    typedef typename std::iterator_traits<T>::difference_type   difference_type;
    //typedef typename std::iterator_traits<T>::distance_type     distance_type;
    typedef typename std::iterator_traits<T>::iterator_category iterator_category;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    block_strided_iterator() {}

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    block_strided_iterator(T vec, const uint32 stride, const uint32 offset = 0) : m_vec( vec ), m_offset(offset), m_stride( stride ) {}

    /// const dereferencing operator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    const_reference operator*() const { return m_vec[m_offset]; }

    /// const indexing operator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    const_reference operator[](const uint32 i) const
    {
        if (LAYOUT == ROW_MAJOR_LAYOUT)
            return m_vec[((i+m_offset) / BLOCKSIZE)*m_stride + ((i+m_offset) % BLOCKSIZE)];
        else
            return m_vec[((i+m_offset) % BLOCKSIZE)*m_stride + ((i+m_offset) / BLOCKSIZE)];
    }

    /// indexing operator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    reference operator[](const uint32 i)
    {
        if (LAYOUT == ROW_MAJOR_LAYOUT)
            return m_vec[((i+m_offset) / BLOCKSIZE)*m_stride + ((i+m_offset) % BLOCKSIZE)];
        else
            return m_vec[((i+m_offset) % BLOCKSIZE)*m_stride + ((i+m_offset) / BLOCKSIZE)];
    }

    /// addition
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    block_strided_iterator<BLOCKSIZE,T> operator+(const uint32 i) const
    {
        return block_strided_iterator<BLOCKSIZE,T>( m_vec, m_stride, m_offset + i );
    }

    /// iterator subtraction
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    difference_type operator-(const block_strided_iterator<BLOCKSIZE,T> it) const
    {
        return (m_vec + m_offset) - (it.m_vec + it.m_offset);
    }

    /// pre-increment
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    block_strided_iterator<BLOCKSIZE,T>& operator++()
    {
        ++m_offset;
        return *this;
    }

    T      m_vec;
    uint32 m_offset;
    uint32 m_stride;
};

/// operator ==
///
template <uint32 BLOCKSIZE,typename T, block_strided_layout LAYOUT>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool operator==(const block_strided_iterator<BLOCKSIZE,T,LAYOUT> it1, const block_strided_iterator<BLOCKSIZE,T,LAYOUT> it2)
{
    return (it1.m_vec == it2.m_vec) && (it1.m_offset == it2.m_offset) && (it1.m_stride == it2.m_stride);
}
/// operator !=
///
template <uint32 BLOCKSIZE,typename T, block_strided_layout LAYOUT>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool operator!=(const block_strided_iterator<BLOCKSIZE,T,LAYOUT> it1, const block_strided_iterator<BLOCKSIZE,T,LAYOUT> it2)
{
    return (it1.m_vec != it2.m_vec) || (it1.m_offset != it2.m_offset) || (it1.m_stride != it2.m_stride);
}

///@} Iterators
///@} Basic

} // namespace nvbio
