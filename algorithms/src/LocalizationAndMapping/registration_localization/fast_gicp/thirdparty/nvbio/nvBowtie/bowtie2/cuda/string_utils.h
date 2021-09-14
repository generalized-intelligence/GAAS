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

#include <nvbio/basic/cached_iterator.h>
#include <nvbio/basic/packedstream.h>

namespace nvbio {
namespace bowtie2 {
namespace cuda {

// helper class to convert {uint32,uint4} to uint32 streams
template <typename Iterator, typename value_type>
struct StreamAdapterBase {};

// helper class to convert uint32 to uint32 streams
template <typename Iterator>
struct StreamAdapterBase<Iterator,uint32>
{
    typedef const_cached_iterator<Iterator> type;

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static type adapt(const Iterator it) { return type( it ); }
};

// helper class to convert uint4 to uint32 streams
template <typename Iterator>
struct StreamAdapterBase<Iterator,uint4>
{
    typedef const_cached_iterator<Iterator>                                         BaseCachedIterator;
    typedef const_cached_iterator< uint4_as_uint32_iterator<BaseCachedIterator> >   type;

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static type adapt(const Iterator it) { return type( uint4_as_uint32_iterator<BaseCachedIterator>( BaseCachedIterator(it) ) ); }
};

// helper class to convert {uint32,uint4} to uint32 streams
template <typename Iterator>
struct StreamAdapter
{
    typedef typename std::iterator_traits<Iterator>::value_type                     value_type;
    typedef StreamAdapterBase<Iterator,value_type>                                  adapter_type;
    typedef typename adapter_type::type                                             type;

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static type adapt(const Iterator it) { return adapter_type::adapt( it ); }
};

} // namespace cuda
} // namespace bowtie2

} // namespace nvbio
