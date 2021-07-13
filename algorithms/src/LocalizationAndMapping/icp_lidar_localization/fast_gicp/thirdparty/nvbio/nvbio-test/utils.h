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

#include <nvbio/basic/packedstream.h>
#include <nvbio/basic/cached_iterator.h>
#include <nvbio/basic/vector_view.h>

namespace nvbio {

template <typename StringType>
struct lmem_selector {};

template <typename StreamType,
          typename SymbolType,
          uint32 SYMBOL_SIZE_T,
          bool   BIG_ENDIAN_T>
struct lmem_selector< vector_view< PackedStream<StreamType,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T> > >
{
    typedef typename std::iterator_traits<StreamType>::value_type type;

    static const uint32 SYMBOLS_PER_WORD = (sizeof(type)*8)/SYMBOL_SIZE_T;
    static const uint32 WORDS            = 512 / SYMBOLS_PER_WORD;
};

template <typename StreamType,
          typename SymbolType,
          uint32 SYMBOL_SIZE_T,
          bool   BIG_ENDIAN_T,
          typename W>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
vector_view< typename PackedStream<const_cached_iterator<const W*>,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T>::iterator >
make_local_string(
    vector_view< PackedStream<StreamType,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T> > string,
    W* lmem)
{
    const StreamType in_stream = string.begin().stream();
    const uint32     in_offset = string.begin().index();
    const uint32     N         = string.length();

    typedef typename std::iterator_traits<StreamType>::value_type word_type;

    const uint32 SYMBOLS_PER_WORD = (sizeof(word_type)*8) / SYMBOL_SIZE_T;
          uint32 word_offset      = in_offset & (SYMBOLS_PER_WORD-1);
          uint32 begin_word       = in_offset / SYMBOLS_PER_WORD;
          uint32 end_word         = (in_offset + N + SYMBOLS_PER_WORD-1) / SYMBOLS_PER_WORD;

    for (uint32 word = begin_word; word < end_word; ++word)
        lmem[word - begin_word] = in_stream[ word ];

    typedef PackedStream<const_cached_iterator<const W*>,uint8,SYMBOL_SIZE_T,BIG_ENDIAN_T> const_stream_type;
    const_stream_type clmem_stream( lmem );

    return vector_view<const_stream_type>( N, clmem_stream + word_offset );
}

} // namespace nvbio
