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

/*! \file string_iterator.h
 *   \brief a generic string iterator, which can be used to build an iterator for any string container
 */

#pragma once

#include <nvbio/basic/types.h>
#include <nvbio/basic/iterator.h>
#include <nvbio/strings/string_traits.h>

namespace nvbio {

///@addtogroup Strings
///@{

///
/// A simple class to build an iterator on top of any string container
///
template <typename StringType>
struct string_iterator
{
    static const uint32 SYMBOL_SIZE = string_traits<StringType>::SYMBOL_SIZE;

    typedef StringType                                          string_type;
    typedef random_access_universal_iterator_tag                iterator_category; // TODO: determine the system from the string_traits
    typedef typename string_traits<StringType>::value_type      value_type;
    typedef typename string_traits<StringType>::reference       reference;
    typedef typename string_traits<StringType>::pointer         pointer;
    typedef typename string_traits<StringType>::index_type      index_type;
    typedef typename signed_type<index_type>::type              difference_type;
    typedef string_iterator<StringType>                         forward_iterator;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE string_iterator()
        : m_index(index_type(-1)) {}

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE string_iterator(const StringType& string, const index_type index = 0)
        : m_string( string ), m_index(index) {}

    /// indexing operator
    ///
    /// \param i        requested value
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE reference operator[] (const index_type i) { return m_string[ m_index + i ]; }

    /// indexing operator
    ///
    /// \param i        requested value
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE reference operator[] (const index_type i) const { return m_string[ m_index + i ]; }

    /// dereference operator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE reference operator* () { return m_string[ m_index ]; }

    /// dereference operator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE reference operator* () const { return m_string[ m_index ]; }

public:
    string_type m_string;
    index_type  m_index;
};

///\relates string_iterator
/// make a string iterator
///
template <typename StringType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
string_iterator<StringType> make_string_iterator(const StringType& string)
{
    return string_iterator<StringType>( string );
}

///\relates string_iterator
/// less than
///
template <typename StringType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool operator< (
    const string_iterator<StringType>& it1,
    const string_iterator<StringType>& it2);

///\relates string_iterator
/// greater than
///
template <typename StringType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool operator> (
    const string_iterator<StringType>& it1,
    const string_iterator<StringType>& it2);

///\relates string_iterator
/// less than
///
template <typename StringType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool operator<= (
    const string_iterator<StringType>& it1,
    const string_iterator<StringType>& it2);

///\relates string_iterator
/// greater than
///
template <typename StringType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool operator>= (
    const string_iterator<StringType>& it1,
    const string_iterator<StringType>& it2);

///\relates string_iterator
/// equality test
///
template <typename StringType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool operator== (
    const string_iterator<StringType>& it1,
    const string_iterator<StringType>& it2);

///\relates string_iterator
/// inequality test
///
template <typename StringType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool operator!= (
    const string_iterator<StringType>& it1,
    const string_iterator<StringType>& it2);

///\relates string_iterator
/// pre-increment operator
///
template <typename StringType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
string_iterator<StringType>& operator++ (string_iterator<StringType>& it);

///\relates string_iterator
/// post-increment operator
///
template <typename StringType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
string_iterator<StringType> operator++ (string_iterator<StringType>& it, int dummy);

///\relates string_iterator
/// pre-decrement operator
///
template <typename StringType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
string_iterator<StringType>& operator-- (string_iterator<StringType>& it);

///\relates string_iterator
/// post-decrement operator
///
template <typename StringType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
string_iterator<StringType> operator-- (string_iterator<StringType>& it, int dummy);

///\relates string_iterator
/// add offset
///
template <typename StringType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
string_iterator<StringType>& operator+= (string_iterator<StringType>& it, const typename string_iterator<StringType>::difference_type distance);

///\relates string_iterator
/// subtract offset
///
template <typename StringType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
string_iterator<StringType>& operator-= (string_iterator<StringType>& it, const typename string_iterator<StringType>::difference_type distance);

///\relates string_iterator
/// add offset
///
template <typename StringType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
string_iterator<StringType> operator+ (const string_iterator<StringType> it, const typename string_iterator<StringType>::difference_type distance);

///\relates string_iterator
/// subtract offset
///
template <typename StringType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
string_iterator<StringType> operator- (const string_iterator<StringType> it, const typename string_iterator<StringType>::difference_type distance);

///\relates string_iterator
/// difference
///
template <typename StringType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
typename string_iterator<StringType>::difference_type operator- (const string_iterator<StringType> it1, const string_iterator<StringType> it2);

///@} Strings

} // namespace nvbio

#include <nvbio/strings/string_iterator_inl.h>
