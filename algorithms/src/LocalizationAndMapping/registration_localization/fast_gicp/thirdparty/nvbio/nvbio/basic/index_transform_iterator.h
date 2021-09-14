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
#include <nvbio/basic/iterator_reference.h>

namespace nvbio {

///@addtogroup Basic
///@{

///@addtogroup Iterators
///@{

///
/// Wrapper class to create a transform iterator out of another base iterator
/// and an index transformation functor
///
template <typename T, typename Transform>
struct index_transform_iterator
{
    typedef index_transform_iterator<T,Transform>               this_type;
    typedef typename Transform::result_type                     value_type;
    typedef iterator_reference<this_type>                       reference;
    typedef value_type                                          const_reference;
    typedef value_type*                                         pointer;
    typedef typename std::iterator_traits<T>::difference_type   difference_type;
    //typedef typename std::iterator_traits<T>::distance_type     distance_type;
    typedef std::random_access_iterator_tag                     iterator_category;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    index_transform_iterator() {}

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    index_transform_iterator(const T base, const Transform f, const difference_type i = 0) : m_base( base ), m_f( f ), m_index( i ) {}

    /// copy constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    index_transform_iterator(const index_transform_iterator& it) : m_base( it.m_base ), m_f( it.m_f ), m_index( it.m_index ) {}

    /// set method
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void set(const value_type v) { m_base[ m_f( m_index ) ] = v; }

    /// const indexing operator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    const_reference operator[](const uint32 i) const { return m_base[ m_f( m_index + i ) ]; }

    /// const indexing operator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    reference operator[](const uint32 i) { return reference( *this + i ); }

    /// dereference operator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    value_type operator*() const { return m_base[ m_f(m_index) ]; }

    /// dereference operator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    reference operator*() { return reference( *this ); }

    /// pre-increment
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    index_transform_iterator<T,Transform>& operator++()
    {
        ++m_index;
        return *this;
    }

    /// post-increment
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    index_transform_iterator<T,Transform> operator++(int i)
    {
        index_transform_iterator<T,Transform> r( m_base, m_f, m_index );
        ++m_index;
        return r;
    }

    /// pre-decrement
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    index_transform_iterator<T,Transform>& operator--()
    {
        --m_index;
        return *this;
    }

    /// post-decrement
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    index_transform_iterator<T,Transform> operator--(int i)
    {
        index_transform_iterator<T,Transform> r( m_base, m_f, m_index );
        --m_index;
        return r;
    }

    /// addition
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    index_transform_iterator<T,Transform> operator+(const difference_type i) const
    {
        return index_transform_iterator( m_base, m_f, m_index + i );
    }

    /// subtraction
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    index_transform_iterator<T,Transform> operator-(const difference_type i) const
    {
        return index_transform_iterator( m_base, m_f, m_index - i );
    }

    /// addition
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    index_transform_iterator<T,Transform>& operator+=(const difference_type i)
    {
        m_index += i;
        return *this;
    }

    /// subtraction
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    index_transform_iterator<T,Transform>& operator-=(const difference_type i)
    {
        m_index -= i;
        return *this;
    }

    /// iterator subtraction
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    difference_type operator-(const index_transform_iterator<T,Transform> it) const
    {
        return m_index - it.m_index;
    }

    /// assignment
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    index_transform_iterator& operator=(const index_transform_iterator<T,Transform>& it)
    {
        m_base  = it.m_base;
        m_f     = it.m_f;
        m_index = it.m_index;
        return *this;
    }

    T               m_base;
    Transform       m_f;
    difference_type m_index;
};

/// make a transform_iterator
///
template <typename T, typename Transform>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
index_transform_iterator<T,Transform> make_index_transform_iterator(const T it, const Transform f)
{
    return index_transform_iterator<T,Transform>( it, f );
}


/// operator ==
///
template <typename T, typename Transform>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool operator==(const index_transform_iterator<T,Transform> it1, const index_transform_iterator<T,Transform> it2)
{
    return (it1.m_base == it2.m_base) && (it1.m_f == it2.m_f) && (it1.m_index == it2.m_index);
}
/// operator !=
///
template <typename T, typename Transform>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool operator!=(const index_transform_iterator<T,Transform> it1, const index_transform_iterator<T,Transform> it2)
{
    return (it1.m_base != it2.m_base) || (it1.m_f != it2.m_f) || (it1.m_index != it2.m_index);
}
/// operator <
///
template <typename T, typename Transform>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool operator<(const index_transform_iterator<T,Transform> it1, const index_transform_iterator<T,Transform> it2) { return (it1.m_index < it2.m_index); }
/// operator <=
///
template <typename T, typename Transform>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool operator<=(const index_transform_iterator<T,Transform> it1, const index_transform_iterator<T,Transform> it2) { return (it1.m_index <= it2.m_index); }
/// operator >
///
template <typename T, typename Transform>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool operator>(const index_transform_iterator<T,Transform> it1, const index_transform_iterator<T,Transform> it2) { return (it1.m_index > it2.m_index); }
/// operator >=
///
template <typename T, typename Transform>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool operator>=(const index_transform_iterator<T,Transform> it1, const index_transform_iterator<T,Transform> it2) { return (it1.m_index >= it2.m_index); }

///@} Iterators
///@} Basic

} // namespace nvbio
