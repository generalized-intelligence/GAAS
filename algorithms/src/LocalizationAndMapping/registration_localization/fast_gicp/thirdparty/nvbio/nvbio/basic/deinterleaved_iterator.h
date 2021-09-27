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

///
/// A helper class to select a component out of a vector of interleaved data.
///
template<uint32 STRIDE, uint32 WHICH, typename BaseIterator>
struct deinterleaved_iterator
{
    typedef typename std::iterator_traits<BaseIterator>::value_type     value_type;
    typedef typename std::iterator_traits<BaseIterator>::reference      reference;
    typedef const value_type*                                           pointer;
    typedef int32                                                       difference_type;
    typedef std::random_access_iterator_tag                             iterator_category;

    typedef deinterleaved_iterator<STRIDE,WHICH,BaseIterator> this_type;

    /// constructor
    ///
    NVBIO_HOST_DEVICE_TEMPLATE
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    deinterleaved_iterator() {}

    /// constructor
    ///
    NVBIO_HOST_DEVICE_TEMPLATE
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    deinterleaved_iterator(const BaseIterator it) : m_it( it ) {}

    /// copy constructor
    ///
    NVBIO_HOST_DEVICE_TEMPLATE
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    deinterleaved_iterator(const deinterleaved_iterator& it) : m_it( it.m_it ) {}

    /// indexing operator
    ///
    NVBIO_HOST_DEVICE_TEMPLATE
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    value_type operator[] (const uint32 i) const { return m_it[ i*STRIDE + WHICH ]; }

    /// indexing operator
    ///
    NVBIO_HOST_DEVICE_TEMPLATE
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    reference operator[] (const uint32 i) { return m_it[ i*STRIDE + WHICH ]; }

    /// dereference operator
    ///
    NVBIO_HOST_DEVICE_TEMPLATE
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    reference operator*() const { return m_it[ WHICH ]; }

    /// pre-increment
    ///
    NVBIO_HOST_DEVICE_TEMPLATE
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    deinterleaved_iterator& operator++()
    {
        ++m_it;
        return *this;
    }

    /// post-increment
    ///
    NVBIO_HOST_DEVICE_TEMPLATE
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    deinterleaved_iterator operator++(int i)
    {
        this_type r( m_it );
        ++m_it;
        return r;
    }

    /// pre-decrement
    ///
    NVBIO_HOST_DEVICE_TEMPLATE
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    deinterleaved_iterator& operator--()
    {
        --m_it;
        return *this;
    }

    /// post-decrement
    ///
    NVBIO_HOST_DEVICE_TEMPLATE
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    deinterleaved_iterator operator--(int i)
    {
        this_type r( m_it );
        --m_it;
        return r;
    }

    /// addition
    ///
    NVBIO_HOST_DEVICE_TEMPLATE
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    deinterleaved_iterator operator+(const difference_type i) const
    {
        return deinterleaved_iterator( m_it + i );
    }

    /// subtraction
    ///
    NVBIO_HOST_DEVICE_TEMPLATE
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    deinterleaved_iterator operator-(const difference_type i) const
    {
        return this_type( m_it - i );
    }

    /// addition
    ///
    NVBIO_HOST_DEVICE_TEMPLATE
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    deinterleaved_iterator& operator+=(const difference_type i)
    {
        m_it += i;
        return *this;
    }

    /// subtraction
    ///
    NVBIO_HOST_DEVICE_TEMPLATE
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    deinterleaved_iterator& operator-=(const difference_type i)
    {
        m_it -= i;
        return *this;
    }

    /// iterator subtraction
    ///
    NVBIO_HOST_DEVICE_TEMPLATE
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    difference_type operator-(const deinterleaved_iterator& it) const
    {
        return m_it - it.m_it;
    }

    /// assignment
    ///
    NVBIO_HOST_DEVICE_TEMPLATE
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    deinterleaved_iterator& operator=(const deinterleaved_iterator& it)
    {
        m_it = it.m_it;
        return *this;
    }

    BaseIterator m_it;
};


} // namespace nvbio
