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
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool operator< (
    const const_cached_iterator<Stream>& it1,
    const const_cached_iterator<Stream>& it2)
{
    return it1.stream() < it2.stream();
}

// less than
//
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool operator<= (
    const const_cached_iterator<Stream>& it1,
    const const_cached_iterator<Stream>& it2)
{
    return it1.stream() <= it2.stream();
}

// greater than
//
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool operator> (
    const const_cached_iterator<Stream>& it1,
    const const_cached_iterator<Stream>& it2)
{
    return it1.stream() > it2.stream();
}

// greater than
//
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool operator>= (
    const const_cached_iterator<Stream>& it1,
    const const_cached_iterator<Stream>& it2)
{
    return it1.stream() >= it2.stream();
}

// equality test
//
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool operator== (
    const const_cached_iterator<Stream>& it1,
    const const_cached_iterator<Stream>& it2)
{
    return it1.stream() == it2.stream();
}

// inequality test
//
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool operator!= (
    const const_cached_iterator<Stream>& it1,
    const const_cached_iterator<Stream>& it2)
{
    return it1.stream() != it2.stream();
}

// pre-increment operator
//
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
const_cached_iterator<Stream>& operator++ (const_cached_iterator<Stream>& it)
{
    ++it.m_stream;
    it.m_cached_idx = uint32(-1);
    return it;
}

// post-increment operator
//
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
const_cached_iterator<Stream> operator++ (const_cached_iterator<Stream>& it, int dummy)
{
    const_cached_iterator<Stream> r( it.m_stream );
    ++it.m_stream;
    it.m_cached_idx = uint32(-1);
    return r;
}

// pre-decrement operator
//
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
const_cached_iterator<Stream>& operator-- (const_cached_iterator<Stream>& it)
{
    --it.m_stream;
    it.m_cached_idx = uint32(-1);
    return it;
}

// post-decrement operator
//
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
const_cached_iterator<Stream> operator-- (const_cached_iterator<Stream>& it, int dummy)
{
    const_cached_iterator<Stream> r( it.m_stream );
    --it.m_stream;
    it.m_cached_idx = uint32(-1);
    return r;
}

// add offset
//
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
const_cached_iterator<Stream>& operator+= (const_cached_iterator<Stream>& it, const typename const_cached_iterator<Stream>::difference_type distance)
{
    it.m_stream += distance;
    it.m_cached_idx = uint32(-1);
    return it;
}

// subtract offset
//
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
const_cached_iterator<Stream>& operator-= (const_cached_iterator<Stream>& it, const typename const_cached_iterator<Stream>::difference_type distance)
{
    it.m_stream -= distance;
    it.m_cached_idx = uint32(-1);
    return it;
}

// add offset
//
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
const_cached_iterator<Stream> operator+ (const const_cached_iterator<Stream> it, const typename const_cached_iterator<Stream>::difference_type distance)
{
    return const_cached_iterator<Stream>( it.stream() + distance );
}

// subtract offset
//
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
const_cached_iterator<Stream> operator- (const const_cached_iterator<Stream> it, const typename const_cached_iterator<Stream>::difference_type distance)
{
    return const_cached_iterator<Stream>( it.stream() - distance );
}

// difference
//
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
typename const_cached_iterator<Stream>::difference_type operator- (const const_cached_iterator<Stream> it1, const const_cached_iterator<Stream> it2)
{
    return it1.stream() - it2.stream();
}

} // namespace nvbio
