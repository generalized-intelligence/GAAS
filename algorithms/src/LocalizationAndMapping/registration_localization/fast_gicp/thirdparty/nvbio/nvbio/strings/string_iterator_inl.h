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

#include <nvbio/basic/types.h>

namespace nvbio {

// less than
//
template <typename StringType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool operator< (
    const string_iterator<StringType>& it1,
    const string_iterator<StringType>& it2)
{
    return it1.m_index < it2.m_index;
}

// less than
//
template <typename StringType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool operator<= (
    const string_iterator<StringType>& it1,
    const string_iterator<StringType>& it2)
{
    return it1.m_index <= it2.m_index;
}

// greater than
//
template <typename StringType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool operator> (
    const string_iterator<StringType>& it1,
    const string_iterator<StringType>& it2)
{
    return it1.m_index > it2.m_index;
}

// greater than
//
template <typename StringType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool operator>= (
    const string_iterator<StringType>& it1,
    const string_iterator<StringType>& it2)
{
    return it1.m_index >= it2.m_index;
}

// equality test
//
template <typename StringType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool operator== (
    const string_iterator<StringType>& it1,
    const string_iterator<StringType>& it2)
{
    return it1.m_index == it2.m_index;
}

// inequality test
//
template <typename StringType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool operator!= (
    const string_iterator<StringType>& it1,
    const string_iterator<StringType>& it2)
{
    return it1.m_index != it2.m_index;
}

// pre-increment operator
//
template <typename StringType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
string_iterator<StringType>& operator++ (string_iterator<StringType>& it)
{
    ++it.m_index;
    return it;
}

// post-increment operator
//
template <typename StringType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
string_iterator<StringType> operator++ (string_iterator<StringType>& it, int dummy)
{
    const string_iterator<StringType> r( it.m_stream );
    ++it.m_index;
    return r;
}

// pre-decrement operator
//
template <typename StringType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
string_iterator<StringType>& operator-- (string_iterator<StringType>& it)
{
    --it.m_index;
    return it;
}

// post-decrement operator
//
template <typename StringType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
string_iterator<StringType> operator-- (string_iterator<StringType>& it, int dummy)
{
    const string_iterator<StringType> r( it.m_stream );
    --it.m_index;
    return r;
}

// add offset
//
template <typename StringType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
string_iterator<StringType>& operator+= (string_iterator<StringType>& it, const typename string_iterator<StringType>::difference_type distance)
{
    it.m_index += distance;
    return it;
}

// subtract offset
//
template <typename StringType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
string_iterator<StringType>& operator-= (string_iterator<StringType>& it, const typename string_iterator<StringType>::difference_type distance)
{
    it.m_index -= distance;
    return it;
}

// add offset
//
template <typename StringType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
string_iterator<StringType> operator+ (const string_iterator<StringType> it, const typename string_iterator<StringType>::difference_type distance)
{
    return string_iterator<StringType>( it.m_string, it.m_index + distance );
}

// subtract offset
//
template <typename StringType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
string_iterator<StringType> operator- (const string_iterator<StringType> it, const typename string_iterator<StringType>::difference_type distance)
{
    return string_iterator<StringType>( it.m_string, it.m_index - distance );
}

// difference
//
template <typename StringType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
typename string_iterator<StringType>::difference_type operator- (const string_iterator<StringType> it1, const string_iterator<StringType> it2)
{
    return it1.m_index - it2.m_index;
}

} // namespace nvbio
