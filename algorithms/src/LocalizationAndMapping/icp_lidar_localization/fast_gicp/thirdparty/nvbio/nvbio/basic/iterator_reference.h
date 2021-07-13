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

///@addtogroup Basic
///@{

///@addtogroup Iterators
///@{

///
/// An iterator reference wrapper, allowing an iterator to return assignable references.
/// Besides the standard iterator interface, the Iterator class must implement a set() method:
///
///\code
/// // set the value of the iterator
/// void set(const value_type v);
///\endcode
///
template <typename Iterator>
struct iterator_reference
{
    typedef typename std::iterator_traits<Iterator>::value_type value_type;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE iterator_reference(Iterator it) : m_it( it ) {}

    /// copy constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE iterator_reference(const iterator_reference& ref) : m_it( ref.m_it ) {}

    /// assignment operator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE iterator_reference& operator= (const iterator_reference& ref) { m_it.set( *ref.m_it ); return *this; }

    /// assignment operator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE iterator_reference& operator= (const value_type s) { m_it.set( s ); return *this; }

    /// conversion operator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE operator value_type() const { return *m_it; }

    Iterator m_it;
};

/// redefine the to_const meta-function for iterator_reference to just return a symbol
///
template <typename Iterator> struct to_const< iterator_reference<Iterator> >
{
    typedef typename iterator_reference<Iterator>::value_type type;
};

///@} Iterators
///@} Basic

} // namespace nvbio

namespace std {

/// overload swap for iterator_reference to make sure it does the right thing
///
template <typename Iterator>
void swap(
    nvbio::iterator_reference<Iterator> ref1,
    nvbio::iterator_reference<Iterator> ref2)
{
    typename nvbio::iterator_reference<Iterator>::value_type tmp = ref1;

    ref1 = ref2;
    ref2 = tmp;
}

} // namespace std
