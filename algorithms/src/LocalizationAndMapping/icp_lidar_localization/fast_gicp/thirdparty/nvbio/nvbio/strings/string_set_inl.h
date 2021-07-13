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

#include <nvbio/basic/algorithms.h>
#include <nvbio/basic/exceptions.h>

#if defined(__CUDACC__)

#include <nvbio/basic/cuda/arch.h>
#include <thrust/device_vector.h>
#include <thrust/scan.h>
#if THRUST_VERSION >= 100700
#include <thrust/execution_policy.h>
#endif

#endif // defined(__CUDACC__)

namespace nvbio {

#if defined(__CUDACC__)

namespace cuda {

//
// A kernel to extract string lengths from a generic string set
//
template <
    uint32   BLOCKDIM,
    typename Iterator,
    typename T>
__global__
void vector_init_kernel(
    const uint32 N,
    Iterator     out,
    T            value)
{
    const uint32 tid = threadIdx.x + blockIdx.x * BLOCKDIM;
    if (tid < N)
        out[tid] = value;
}

//
// A kernel to extract string lengths from a generic string set
//
template <
    uint32   BLOCKDIM,
    typename InStringSet,
    typename OutLengthIterator>
__global__
void generic_string_lengths_kernel(
    const uint32      N_strings,
    const InStringSet in_set,
    OutLengthIterator out_lengths)
{
    const uint32 tid = threadIdx.x + blockIdx.x * BLOCKDIM;
    if (tid > N_strings) // allow to fill one more entry than N_strings
        return;

    out_lengths[ tid ] = tid < N_strings ? in_set[tid].size() : 0u;
}

//
// A kernel to transform a generic string set into a concatenated set
//
template <
    uint32   BLOCKDIM,
    typename InStringSet,
    typename OutStringIterator,
    typename OutOffsetIterator>
__global__
void generic_to_concat_kernel(
    const uint32      N_strings,
    InStringSet       in_set,
    OutStringIterator out_string,
    OutOffsetIterator out_offsets)
{
    const uint32 tid = threadIdx.x + blockIdx.x * BLOCKDIM;
    if (tid >= N_strings)
        return;

    typename InStringSet::string_type in_string = in_set[tid];

    const uint32 length = in_string.size();

    const uint32 offset = out_offsets[tid];
    for (uint32 j = 0; j < length; ++j)
        out_string[offset + j] = in_string[j];
}

//
// A kernel to transform a sparse/concatenated string set into a concatenated set
// Use one warp per string.
//
template <
    uint32   BLOCKDIM,
    typename InStringSet,
    typename OutStringIterator,
    typename OutOffsetIterator>
__global__
void contig_to_concat_kernel(
    const uint32      N_strings,
    InStringSet       in_set,
    OutStringIterator out_string,
    OutOffsetIterator out_offsets)
{
    const uint32 NUM_WARPS = BLOCKDIM >> cuda::Arch::LOG_WARP_SIZE;
    const uint32 wid = warp_id() + blockIdx.x * NUM_WARPS;
    if (wid >= N_strings)
        return;

    typename InStringSet::string_type in_string = in_set[wid];

    const uint32 length = in_string.size();

    const uint32 offset = out_offsets[wid];

    for (uint32 j = warp_tid(); j < length; j += cuda::Arch::WARP_SIZE)
        out_string[offset + j] = in_string[j];
}

//
// A kernel to read a bunch of values from a generic iterator and write them to another generic iterator
//
template <
    uint32   BLOCKDIM,
    typename InputIterator,
    typename OutputIterator>
__global__
void generic_vector_copy_kernel(
    const uint32      N,
    InputIterator     input,
    OutputIterator    output)
{
    const uint32 tid = threadIdx.x + blockIdx.x * BLOCKDIM;
    if (tid < N)
        output[tid] = input[tid];
}

//
// A kernel to transform a generic string set into a packed concatenated set.
// This specialization is needed because writing packed concatenated strings
// in parallel is not safe, as some words might be spanned by multiple strings.
//
template <
    uint32   BLOCKDIM,
    uint32   SYMBOL_SIZE,
    bool     BIG_ENDIAN,
    typename InStringSet,
    typename OutStreamIterator,
    typename OutOffsetIterator>
__global__
void generic_to_packed_concat_kernel(
    const uint32      N_strings,
    const uint32      N_words,
    InStringSet       in_set,
    OutStreamIterator out_stream,
    OutOffsetIterator out_offsets)
{
    const uint32 WARP_SIZE        = cuda::Arch::WARP_SIZE;
    const uint32 WORDS_PER_THREAD = 4u;

    const uint32 tid = threadIdx.x + blockIdx.x * BLOCKDIM;
    if (tid * WORDS_PER_THREAD >= N_words)
        return;

    //
    // each thread is responsible to write out WARP_SIZE words
    //
    const uint32 SYMBOLS_PER_WORD = (8u*sizeof(uint32)) / SYMBOL_SIZE;

    // loop through the symbols in the given word and find out which string
    // they belong to in the input set

    uint32 global_symbol = SYMBOLS_PER_WORD * WORDS_PER_THREAD * tid;
    uint32 string_id     = uint32( upper_bound( global_symbol, out_offsets, N_strings ) - out_offsets ) - 1u;
    uint32 local_symbol  = global_symbol - out_offsets[string_id];

    // fetch the relative input string
    typename InStringSet::string_type in_string = in_set[string_id];

    const uint32 wid  = warp_id();
    const uint32 wtid = warp_tid();

    __shared__ volatile uint32 sm[ BLOCKDIM * WORDS_PER_THREAD ];
    volatile uint32* warp_sm = sm + wid * WARP_SIZE * WORDS_PER_THREAD;

    for (uint32 w = 0; w < WORDS_PER_THREAD && string_id < N_strings; ++w)
    {
        uint32 word = 0u;

        #pragma unroll
        for (uint32 s = 0; s < SYMBOLS_PER_WORD; ++s, ++global_symbol, ++local_symbol)
        {
            // compute the local position of the symbol in the input string
            while (local_symbol >= in_string.size())
            {
                in_string    = in_set[++string_id];
                local_symbol = 0;
            }
            if (string_id == N_strings)
                break;

            // fetch the relative character
            const uint8 in_c = in_string[ local_symbol ];

            // and write it into the word
            word |= (in_c << (s*SYMBOL_SIZE));  // TODO: consider endianness here
        }

        // write out the packed word
        warp_sm[ wtid*WORDS_PER_THREAD + w ] = word;
    }

    // and now write the words from each thread in parallel
    const uint32 base_offset = blockIdx.x*BLOCKDIM*WORDS_PER_THREAD +
                               WORDS_PER_THREAD * WARP_SIZE * wid;

    for (uint32 t = 0; t < WARP_SIZE; ++t)
    {
        if (wtid < WORDS_PER_THREAD)
            out_stream[ base_offset + t*WORDS_PER_THREAD + wtid ] =
            warp_sm[ t*WORDS_PER_THREAD + wtid ];
    }
}

//
// A kernel to transform a concatenated string set into a packed concatenated set.
//
template <
    uint32   BLOCKDIM,
    uint32   SYMBOL_SIZE,
    bool     BIG_ENDIAN,
    typename InStringSet,
    typename OutStreamIterator,
    typename OutOffsetIterator>
__global__
void concat_to_packed_concat_kernel(
    const uint32      N_strings,
    const uint32      N_symbols,
    InStringSet       in_set,
    OutStreamIterator out_stream,
    OutOffsetIterator out_offsets)
{
    const uint32 tid = threadIdx.x + blockIdx.x * BLOCKDIM;

    //
    // each thread is responsible to read in 1 word worth of symbols
    //

    // copy the offsets
    if (tid <= N_strings)
        out_offsets[tid] = in_set.offsets()[tid];

    const uint32 SYMBOLS_PER_WORD = (8u*sizeof(uint32)) / SYMBOL_SIZE;
    if (tid * SYMBOLS_PER_WORD >= N_symbols)
        return;

    // fetch the input symbol string
    typename InStringSet::symbol_iterator in_symbols = in_set.base_string();

    uint32 word = 0;
    #pragma unroll
    for (uint32 s = 0, in_s = tid * SYMBOLS_PER_WORD; s < SYMBOLS_PER_WORD; ++s, ++in_s)
    {
        const uint8 in_c = in_s < N_symbols ? in_symbols[ in_s ] : 0u;
        word |= in_c << (s*SYMBOL_SIZE); // TODO: handle Endianness here
    }

    // write the packed word out
    out_stream[ tid ] = word;
}

//
// A kernel to transform a generic string set into a strided set
//
template <
    uint32   BLOCKDIM,
    typename InStringSet,
    typename OutStringIterator,
    typename OutLengthIterator>
__global__
void generic_to_strided_kernel(
    const uint32      N_strings,
    const uint32      out_stride,
    InStringSet       in_set,
    OutStringIterator out_stream,
    OutLengthIterator out_lengths)
{
    const uint32 tid = threadIdx.x + blockIdx.x * BLOCKDIM;
    if (tid >= N_strings)
        return;

    typename InStringSet::string_type in_string = in_set[tid];

    const uint32 length = in_string.size();

    typedef strided_iterator<OutStringIterator> strided_stream_type;

    strided_stream_type out_string( out_stream + tid, out_stride );

    for (uint32 j = 0; j < length; ++j)
        out_string[j] = in_string[j];

    if (tid < N_strings)
        out_lengths[tid] = length;
}

//
// A kernel to transform a packed-sparse string set into a strided set
//
template <
    uint32 BLOCKDIM,
    uint32 BITS,
    bool   BIG_ENDIAN,
    typename InStreamIterator,
    typename InOffsetIterator,
    typename OutStringIterator,
    typename OutLengthIterator>
__global__
void packed_concat_to_strided_kernel(
    const uint32      N_strings,
    const uint32      out_stride,
    InStreamIterator  in_stream,
    InOffsetIterator  in_offsets,
    OutStringIterator out_stream,
    OutLengthIterator out_lengths)
{
    const uint32 tid = threadIdx.x + blockIdx.x * BLOCKDIM;
    if (tid >= N_strings)
        return;

    const uint32 in_offset = in_offsets[tid];
    const uint32 N         = in_offsets[tid+1] - in_offset;

#if 0
    typedef typename std::iterator_traits<InStreamIterator>::value_type word_type;

    const uint32 SYMBOLS_PER_WORD = (sizeof(word_type)*8) / BITS;
          uint32 begin_word       = in_offset / SYMBOLS_PER_WORD;
          uint32 end_word         = (in_offset + N + SYMBOLS_PER_WORD-1) / SYMBOLS_PER_WORD;
          uint32 word_offset      = in_offset & (SYMBOLS_PER_WORD-1);

    // load the words of the input stream in local memory with a tight loop
    uint32 lmem[64];
    for (uint32 word = begin_word; word < end_word; ++word)
        lmem[word - begin_word] = in_stream[ word ];

    typedef PackedStream<const_cached_iterator<const uint32*>,uint8,BITS,BIG_ENDIAN> const_stream_type;
    const_stream_type clmem_stream( &lmem[0] );

    // write out the output symbols
    for (uint32 i = 0; i < N; ++i)
        out_stream[ tid + out_stride*i ] = clmem_stream[i + word_offset];
#else
    // Naive
    typedef PackedStream<const_cached_iterator<InStreamIterator>,uint8,BITS,BIG_ENDIAN> const_stream_type;
    const_stream_type cstream( in_stream );

    for (uint32 i = 0; i < N; ++i)
        out_stream[ tid + out_stride*i ] = cstream[i + in_offset];
#endif
    out_lengths[tid] = N;
}

//
// A kernel to transform a packed-sparse string set into a strided set
//
template <
    uint32 BLOCKDIM,
    uint32 BITS,
    bool   BIG_ENDIAN,
    typename InStreamIterator,
    typename InRangeIterator,
    typename OutStringIterator,
    typename OutLengthIterator>
__global__
void packed_sparse_to_strided_kernel(
    const uint32      N_strings,
    const uint32      out_stride,
    InStreamIterator  in_stream,
    InRangeIterator   in_ranges,
    OutStringIterator out_stream,
    OutLengthIterator out_lengths)
{
    const uint32 tid = threadIdx.x + blockIdx.x * BLOCKDIM;
    if (tid >= N_strings)
        return;

    const uint2  range     = in_ranges[tid];
    const uint32 in_offset = range.x;
    const uint32 N         = range.y - in_offset;

#if 1
    typedef typename std::iterator_traits<InStreamIterator>::value_type word_type;

    const uint32 SYMBOLS_PER_WORD = (sizeof(word_type)*8) / BITS;
          uint32 word_offset      = in_offset & (SYMBOLS_PER_WORD-1);
          uint32 begin_word       = in_offset / SYMBOLS_PER_WORD;
          uint32 end_word         = (in_offset + N + SYMBOLS_PER_WORD-1) / SYMBOLS_PER_WORD;

    word_type lmem[64];
    for (uint32 word = begin_word; word < end_word; ++word)
        lmem[word - begin_word] = in_stream[ word ];

    typedef PackedStream<const_cached_iterator<const word_type*>,uint8,BITS,BIG_ENDIAN> const_stream_type;
    const_stream_type clmem_stream( &lmem[0] );

    // write out the output symbols
    for (uint32 i = 0; i < N; ++i)
        out_stream[ tid + out_stride*i ] = clmem_stream[i + word_offset];
#else
    // Naive
    typedef PackedStream<const_cached_iterator<InStreamIterator>,uint8,BITS,BIG_ENDIAN> const_stream_type;
    const_stream_type cstream( in_stream );

    for (uint32 i = 0; i < N; ++i)
        out_stream[ tid + out_stride*i ] = cstream[i + in_offset];
#endif
    out_lengths[tid] = N;
}

//
// A kernel to transform a contiguous string set into a strided set
//
template <
    uint32   BLOCKDIM,
    typename InStringSet,
    typename OutLengthIterator>
__global__
void contig_to_strided_uint8_kernel(
    const uint32      N_strings,
    const uint32      out_stride,
    InStringSet       in_set,
    uint8*            out_stream,
    OutLengthIterator out_lengths)
{
    const uint32 tid = threadIdx.x + blockIdx.x * BLOCKDIM;

    const uint32 length = tid < N_strings ? in_set[ tid ].size() : 0u;

    const uint32 WARP_SIZE = cuda::Arch::WARP_SIZE;
    const uint32 wid  = warp_id();
    const uint32 wtid = warp_tid();

    __shared__ volatile uint8 sm[ BLOCKDIM * WARP_SIZE ];
    volatile uint8* warp_sm = sm + wid * WARP_SIZE * WARP_SIZE;


    // each warp fetches WARP_SIZE characters from WARP_SIZE strings
    for (uint32 block = 0; __any( block < length ); block += WARP_SIZE)
    {
        for (uint32 t = 0; t < WARP_SIZE; ++t)
        {
            // compute the t-th string id
            const uint32 t_string_id = blockIdx.x*BLOCKDIM + wid*WARP_SIZE + t;
            if (t_string_id >= N_strings)
                break;

            // fetch the t-th string
            typename InStringSet::string_type t_string = in_set[ t_string_id ];

            // read 1 symbol per thread
            warp_sm[ wtid*WARP_SIZE + t ] = (block + wtid < t_string.size()) ? t_string[block + wtid] : 0u;
        }

        // at this point we have WARP_SIZE characters from WARP_SIZE adjacent strings in warp_sm: let's write them out
        if (block + WARP_SIZE <= length)
        {
            for (uint32 s = 0; s < WARP_SIZE; ++s)
                out_stream[ tid + (block + s)*out_stride ] = warp_sm[ s*WARP_SIZE + wtid ];
        }
        else if (block < length)
        {
            for (uint32 s = 0; s < WARP_SIZE; ++s)
            {
                if (block + s < length)
                    out_stream[ tid + (block + s)*out_stride ] = warp_sm[ s*WARP_SIZE + wtid ];
            }
        }
    }

    // write out the length
    if (tid < N_strings)
        out_lengths[tid] = length;
}

//
// A kernel to transform a generic string set into a strided set
//
template <
    uint32   BLOCKDIM,
    typename InStringSet,
    typename OutLengthIterator>
__global__
void generic_to_strided_uint8_kernel(
    const uint32      N_strings,
    const uint32      out_stride,
    InStringSet       in_set,
    uint8*            out_stream,
    OutLengthIterator out_lengths)
{
    const uint32 tid = threadIdx.x + blockIdx.x * BLOCKDIM;
    const uint32 base_id = tid*4;
    if (base_id >= N_strings)
        return;

    typedef strided_iterator<uint32*> strided_stream_type;

    strided_stream_type out_string( (uint32*)(out_stream) + tid, out_stride/4u );

    if (base_id + 3 < N_strings)
    {
        // fetch 4 strings
        typename InStringSet::string_type in_string0 = in_set[base_id + 0];
        typename InStringSet::string_type in_string1 = in_set[base_id + 1];
        typename InStringSet::string_type in_string2 = in_set[base_id + 2];
        typename InStringSet::string_type in_string3 = in_set[base_id + 3];

        const uint32 length0 = in_string0.size();
        const uint32 length1 = in_string1.size();
        const uint32 length2 = in_string2.size();
        const uint32 length3 = in_string3.size();

        const uint32 min_length = nvbio::min(
            nvbio::min( length0, length1 ),
            nvbio::min( length2, length3 ) );

        for (uint32 j = 0; j < min_length; ++j)
        {
            const uint32 c0 = in_string0[j];
            const uint32 c1 = in_string1[j];
            const uint32 c2 = in_string2[j];
            const uint32 c3 = in_string3[j];

            out_string[j] = c0 | (c1 << 8) | (c2 << 16) | (c3 << 24);
        }

        const uint32 max_length = nvbio::max(
            nvbio::max( length0, length1 ),
            nvbio::max( length2, length3 ) );

        for (uint32 j = min_length; j < max_length; ++j)
        {
            const uint32 c0 = j < length0 ? in_string0[j] : 0xFFu;
            const uint32 c1 = j < length1 ? in_string1[j] : 0xFFu;
            const uint32 c2 = j < length2 ? in_string2[j] : 0xFFu;
            const uint32 c3 = j < length3 ? in_string3[j] : 0xFFu;

            out_string[j] = c0 | (c1 << 8) | (c2 << 16) | (c3 << 24);
        }

        out_lengths[base_id]     = length0;
        out_lengths[base_id + 1] = length1;
        out_lengths[base_id + 2] = length2;
        out_lengths[base_id + 3] = length3;
    }
    else if (base_id + 2 < N_strings)
    {
        // fetch 3 strings
        typename InStringSet::string_type in_string0 = in_set[base_id + 0];
        typename InStringSet::string_type in_string1 = in_set[base_id + 1];
        typename InStringSet::string_type in_string2 = in_set[base_id + 2];

        const uint32 length0 = in_string0.size();
        const uint32 length1 = in_string1.size();
        const uint32 length2 = in_string2.size();

        const uint32 min_length = nvbio::min3( length0, length1, length2 );
        for (uint32 j = 0; j < min_length; ++j)
        {
            const uint32 c0 = in_string0[j];
            const uint32 c1 = in_string1[j];
            const uint32 c2 = in_string2[j];
            const uint32 c3 = 0xFFu;

            out_string[j] = c0 | (c1 << 8) | (c2 << 16) | (c3 << 24);
        }

        const uint32 max_length = nvbio::max3( length0, length1, length2 );
        for (uint32 j = min_length; j < max_length; ++j)
        {
            const uint32 c0 = j < length0 ? in_string0[j] : 0xFFu;
            const uint32 c1 = j < length1 ? in_string1[j] : 0xFFu;
            const uint32 c2 = j < length2 ? in_string2[j] : 0xFFu;
            const uint32 c3 = 0xFFu;

            out_string[j] = c0 | (c1 << 8) | (c2 << 16) | (c3 << 24);
        }

        out_lengths[base_id]     = length0;
        out_lengths[base_id + 1] = length1;
        out_lengths[base_id + 2] = length2;
    }
    else if (base_id + 1 < N_strings)
    {
        // fetch 2 strings
        typename InStringSet::string_type in_string0 = in_set[base_id + 0];
        typename InStringSet::string_type in_string1 = in_set[base_id + 1];

        const uint32 length0 = in_string0.size();
        const uint32 length1 = in_string1.size();

        const uint32 min_length = nvbio::min( length0, length1 );
        for (uint32 j = 0; j < min_length; ++j)
        {
            const uint32 c0 = in_string0[j];
            const uint32 c1 = in_string1[j];
            const uint32 c2 = 0xFFu;
            const uint32 c3 = 0xFFu;

            out_string[j] = c0 | (c1 << 8) | (c2 << 16) | (c3 << 24);
        }

        const uint32 max_length = nvbio::max( length0, length1 );
        for (uint32 j = min_length; j < max_length; ++j)
        {
            const uint32 c0 = j < length0 ? in_string0[j] : 0xFFu;
            const uint32 c1 = j < length1 ? in_string1[j] : 0xFFu;
            const uint32 c2 = 0xFFu;
            const uint32 c3 = 0xFFu;

            out_string[j] = c0 | (c1 << 8) | (c2 << 16) | (c3 << 24);
        }

        out_lengths[base_id]     = length0;
        out_lengths[base_id + 1] = length1;
    }
    else
    {
        // fetch 1 string
        typename InStringSet::string_type in_string = in_set[base_id + 0];

        const uint32 length = in_string.size();

        for (uint32 j = 0; j < length; ++j)
        {
            const uint32 c0 = in_string[j];
            const uint32 c1 = 0xFFu;
            const uint32 c2 = 0xFFu;
            const uint32 c3 = 0xFFu;

            out_string[j] = c0 | (c1 << 8) | (c2 << 16) | (c3 << 24);
        }

        out_lengths[base_id] = length;
    }
}

//
// A kernel to transform a strided-packed string set into a strided set
//
template <
    uint32   BLOCKDIM,
    typename InStringSet,
    typename OutLengthIterator>
__global__
void strided_packed_to_strided_uint8_kernel(
    const uint32      N_strings,
    const uint32      out_stride,
    InStringSet       in_set,
    uint8*            out_stream,
    OutLengthIterator out_lengths)
{
    const uint32 tid = blockIdx.x * BLOCKDIM + threadIdx.x;
    const uint32 id = tid * 4;
    if (id >= N_strings)
        return;

    __shared__ uint32 sm[ BLOCKDIM*4 ];

    typedef PackedStream<
        uint32*,
        typename InStringSet::symbol_type,
        InStringSet::SYMBOL_SIZE,
        InStringSet::BIG_ENDIAN>               stream_type;

    const uint32 SYMBOLS_PER_WORD = (sizeof(uint32)*8u) / InStringSet::SYMBOL_SIZE;

    const uint32 length0 = in_set.lengths()[id + 0];
    const uint32 length1 = in_set.lengths()[id + 1];
    const uint32 length2 = in_set.lengths()[id + 2];
    const uint32 length3 = in_set.lengths()[id + 3];

    const uint32 N = nvbio::max(
        nvbio::max( length0, length1 ),
        nvbio::max( length2, length3 ) );

    const uint32 N_words = (N + SYMBOLS_PER_WORD-1) / SYMBOLS_PER_WORD;

    const uint32* in_stream = in_set.base_stream();
    const uint32  in_stride = in_set.stride();

    for (uint32 i = 0; i < N_words; ++i)
    {
        // fetch 4 words, 1 from each stream
        const uint4 in_words = *reinterpret_cast<const uint4*>( in_stream + i * in_stride + id );

        // setup 4 streams on the words just read
        sm[threadIdx.x + BLOCKDIM*0] = in_words.x;
        sm[threadIdx.x + BLOCKDIM*1] = in_words.y;
        sm[threadIdx.x + BLOCKDIM*2] = in_words.z;
        sm[threadIdx.x + BLOCKDIM*3] = in_words.w;

        stream_type streams[4] =
        {
            stream_type( &sm[threadIdx.x + BLOCKDIM*0] ),
            stream_type( &sm[threadIdx.x + BLOCKDIM*1] ),
            stream_type( &sm[threadIdx.x + BLOCKDIM*2] ),
            stream_type( &sm[threadIdx.x + BLOCKDIM*3] )
        };

        // write out all the symbols packed in the fetched words interleaving them as uint8's in a simd4u8.
        for (uint32 j = 0; j < SYMBOLS_PER_WORD; ++j)
        {
            // read the next symbol from each of the 4 streams
            const uint32 word =
                (streams[0][j] <<  0) |
                (streams[1][j] <<  8) |
                (streams[2][j] << 16) |
                (streams[3][j] << 24);

            // and write the newly packed word out
            *reinterpret_cast<uint32*>( out_stream + (i*SYMBOLS_PER_WORD + j)*out_stride + id ) = word;
        }
    }

    out_lengths[id]     = length0;
    out_lengths[id + 1] = length1;
    out_lengths[id + 2] = length2;
    out_lengths[id + 3] = length3;
}

//
// A kernel to transform a generic string set into a strided packed set
//
template <
    uint32   BLOCKDIM,
    uint32   SYMBOL_SIZE,
    bool     BIG_ENDIAN,
    typename InStringSet,
    typename OutStreamIterator,
    typename OutLengthIterator>
__global__
void generic_to_strided_packed_kernel(
    const uint32      N_strings,
    const uint32      out_stride,
    InStringSet       in_set,
    OutStreamIterator out_stream,
    OutLengthIterator out_lengths)
{
    const uint32 tid = threadIdx.x + blockIdx.x * BLOCKDIM;
    if (tid >= N_strings)
        return;

    typename InStringSet::string_type in_string = in_set[tid];

    const uint32 length = in_string.size();

    typedef strided_iterator<OutStreamIterator> strided_stream_type;
    typedef PackedStream<strided_stream_type,uint8,SYMBOL_SIZE,BIG_ENDIAN> packed_stream_type;

    packed_stream_type out_string( strided_stream_type( out_stream + tid, out_stride ) );

    for (uint32 j = 0; j < length; ++j)
        out_string[j] = in_string[j];

    if (tid < N_strings)
        out_lengths[tid] = length;
}

//
// A kernel to transform a contiguous string set into a strided packed set
//
template <
    uint32   BLOCKDIM,
    uint32   SYMBOL_SIZE,
    bool     BIG_ENDIAN,
    typename InStringSet,
    typename OutStreamIterator,
    typename OutLengthIterator>
__global__
void contig_to_strided_packed_kernel(
    const uint32      N_strings,
    const uint32      out_stride,
    InStringSet       in_set,
    OutStreamIterator out_stream,
    OutLengthIterator out_lengths)
{
    const uint32 tid = threadIdx.x + blockIdx.x * BLOCKDIM;

    const uint32 length = tid < N_strings ? in_set[ tid ].size() : 0u;

    const uint32 WARP_SIZE = cuda::Arch::WARP_SIZE;
    const uint32 wid  = warp_id();
    const uint32 wtid = warp_tid();

    __shared__ volatile uint8 sm[ BLOCKDIM * WARP_SIZE ];
    volatile uint8* warp_sm = sm + wid * WARP_SIZE * WARP_SIZE;

    typedef strided_iterator<OutStreamIterator> strided_stream_type;
    typedef PackedStream<strided_stream_type,uint8,SYMBOL_SIZE,BIG_ENDIAN> packed_stream_type;

    packed_stream_type out_string( strided_stream_type( out_stream + tid, out_stride ) );


    // each warp fetches WARP_SIZE characters from WARP_SIZE strings
    for (uint32 block = 0; __any( block < length ); block += WARP_SIZE)
    {
        for (uint32 t = 0; t < WARP_SIZE; ++t)
        {
            // compute the t-th string id
            const uint32 t_string_id = blockIdx.x*BLOCKDIM + wid*WARP_SIZE + t;
            if (t_string_id >= N_strings)
                break;

            // fetch the t-th string
            typename InStringSet::string_type t_string = in_set[ t_string_id ];

            // read 1 symbol per thread
            warp_sm[ t*WARP_SIZE + wtid ] = (block + wtid < t_string.size()) ? t_string[block + wtid] : 0u;
        }

        // at this point we have WARP_SIZE characters from WARP_SIZE adjacent strings in warp_sm: let's write them out
        if (block < length)
        {
            //for (uint32 s = 0; s < WARP_SIZE; ++s)
            //{
            //    if (block + s < length)
            //        out_string[ block + s ] = warp_sm[ wtid*WARP_SIZE + s ];
            //}

            // pack the symbols in a word in a register before writing it out
            uint32 word;
            PackedStream<uint32*,uint8,SYMBOL_SIZE,BIG_ENDIAN> packed_word( &word );

            const uint32 SYMBOLS_PER_WORD = (8u*sizeof(uint32))/SYMBOL_SIZE;
            for (uint32 s = 0; s < WARP_SIZE; s += SYMBOLS_PER_WORD)
            {
                if (block + s < length)
                {
                    for (uint32 b = 0; b < SYMBOLS_PER_WORD; ++b)
                        packed_word[b] = warp_sm[ wtid*WARP_SIZE + s + b ];

                    const uint32 word_idx = (block + s) / SYMBOLS_PER_WORD;
                    out_stream[ tid + word_idx*out_stride ] = word;
                }
            }
        }
    }

    // write out the length
    if (tid < N_strings)
        out_lengths[tid] = length;
}

//
// A kernel to transform a packed concatenated string set into a strided one
//
template <
    uint32 BLOCKDIM,
    uint32 SYMBOL_SIZE,
    bool   BIG_ENDIAN,
    typename InStreamIterator,
    typename InOffsetIterator,
    typename OutStreamIterator,
    typename OutLengthIterator>
__global__
void packed_concatenated_to_strided_packed_kernel(
    const uint32      N_strings,
    const uint32      out_stride,
    InStreamIterator  in_stream,
    InOffsetIterator  in_offsets,
    OutStreamIterator out_stream,
    OutLengthIterator out_lengths)
{
    const uint32 tid = threadIdx.x + blockIdx.x * BLOCKDIM;
    if (tid >= N_strings)
        return;

    const uint32 offset = in_offsets[tid];
    const uint32 length = in_offsets[tid+1] - offset;

    transpose_packed_streams<BLOCKDIM,SYMBOL_SIZE,BIG_ENDIAN>(
        out_stride,
        length,
        offset,
        in_stream,
        out_stream + tid );

    out_lengths[tid] = length;
}

//
// A kernel to transform a packed sparse string set into a strided one
//
template <
    uint32 BLOCKDIM,
    uint32 SYMBOL_SIZE,
    bool   BIG_ENDIAN,
    typename InStreamIterator,
    typename InRangeIterator,
    typename OutStreamIterator,
    typename OutLengthIterator>
__global__
void packed_sparse_to_strided_packed_kernel(
    const uint32      N_strings,
    const uint32      out_stride,
    InStreamIterator  in_stream,
    InRangeIterator   in_ranges,
    OutStreamIterator out_stream,
    OutLengthIterator out_lengths)
{
    const uint32 tid = threadIdx.x + blockIdx.x * BLOCKDIM;
    if (tid >= N_strings)
        return;

    const uint2  range  = in_ranges[tid];
    const uint32 offset = range.x;
    const uint32 length = range.y - range.x;

    transpose_packed_streams<BLOCKDIM,SYMBOL_SIZE,BIG_ENDIAN>(
        out_stride,
        length,
        offset,
        in_stream,
        out_stream + tid );

    out_lengths[tid] = length;
}

template <typename OutStringSet>
struct copy_dispatch
{
    template <typename InStringSet>
    struct source_dispatch
    {
        static void enact(
            const InStringSet&  in_string_set,
                  OutStringSet& out_string_set)
        {
        }
    };
};

//
// concatenated output set
//
template <
    typename OutStringIterator,
    typename OutOffsetIterator>
struct copy_dispatch<
    ConcatenatedStringSet<OutStringIterator,OutOffsetIterator>
    >
{
    typedef ConcatenatedStringSet<OutStringIterator,OutOffsetIterator> out_string_set_type;

    // generic input set
    template <typename in_string_set_type>
    struct source_dispatch
    {
        static void enact(
            const in_string_set_type&  in_string_set,
                  out_string_set_type& out_string_set)
        {
            const uint32 BLOCKDIM = 64u;

            if (out_string_set.size() != in_string_set.size())
                throw nvbio::runtime_error( "copy() : unmatched string set sizes" );

            const uint32 n_blocks = (in_string_set.size()+1 + BLOCKDIM-1)/BLOCKDIM;

            // extract the string lengths
            generic_string_lengths_kernel<BLOCKDIM> <<<n_blocks,BLOCKDIM>>>(
                in_string_set.size(),
                in_string_set,
                out_string_set.offsets() );

            cudaThreadSynchronize();

        #if THRUST_VERSION <= 100503
            // perform an exclusive scan on the string lengths to get the offsets
            thrust::detail::backend::cuda::exclusive_scan(
                out_string_set.offsets(),
                out_string_set.offsets() + in_string_set.size()+1,
                out_string_set.offsets(),
                0u,
                add_functor() );
        #else
            // perform an exclusive scan on the string lengths to get the offsets
            thrust::exclusive_scan(
                thrust::device,
                out_string_set.offsets(),
                out_string_set.offsets() + in_string_set.size()+1,
                out_string_set.offsets(),
                0u,
                add_functor() );
        #endif

            generic_to_concat_kernel<BLOCKDIM> <<<n_blocks,BLOCKDIM>>>(
                in_string_set.size(),
                in_string_set,
                out_string_set.base_string(),
                out_string_set.offsets() );

            cudaThreadSynchronize();
        }
    };

    // sparse input set
    template <
        typename InStringIterator,
        typename InRangeIterator>
    struct source_dispatch< SparseStringSet<InStringIterator,InRangeIterator> >
    {
        typedef SparseStringSet<InStringIterator,InRangeIterator> in_string_set_type;

        static void enact(
            const in_string_set_type&  in_string_set,
                  out_string_set_type& out_string_set)
        {
            const uint32 BLOCKDIM = 128u;

            if (out_string_set.size() != in_string_set.size())
                throw nvbio::runtime_error( "copy() : unmatched string set sizes" );

            const uint32 n_blocks = (in_string_set.size()+1 + BLOCKDIM-1)/BLOCKDIM;

            // extract the string lengths
            generic_string_lengths_kernel<BLOCKDIM> <<<n_blocks,BLOCKDIM>>>(
                in_string_set.size(),
                in_string_set,
                out_string_set.offsets() );

            cudaThreadSynchronize();

        #if THRUST_VERSION <= 100503
            // perform an exclusive scan on the string lengths to get the offsets
            thrust::detail::backend::cuda::exclusive_scan(
                out_string_set.offsets(),
                out_string_set.offsets() + in_string_set.size()+1,
                out_string_set.offsets(),
                0u,
                add_functor() );
        #else
            // perform an exclusive scan on the string lengths to get the offsets
            thrust::exclusive_scan(
                thrust::device,
                out_string_set.offsets(),
                out_string_set.offsets() + in_string_set.size()+1,
                out_string_set.offsets(),
                0u,
                add_functor() );
        #endif
            {
                // use 1 warp per string
                const uint32 WARPS_PER_BLOCK = BLOCKDIM >> cuda::Arch::LOG_WARP_SIZE;
                const uint32 n_blocks = (in_string_set.size()+1 + WARPS_PER_BLOCK-1)/WARPS_PER_BLOCK;

                contig_to_concat_kernel<BLOCKDIM> <<<n_blocks,BLOCKDIM>>>(
                    in_string_set.size(),
                    in_string_set,
                    out_string_set.base_string(),
                    out_string_set.offsets() );

                cudaThreadSynchronize();
            }
        }
    };

    template <typename in_string_set_type>
    static void enact(
            const in_string_set_type&  in_string_set,
                  out_string_set_type& out_string_set)
    {
        return source_dispatch<in_string_set_type>::enact( in_string_set, out_string_set );
    }
};

//
// packed-concatenated output set
//
template <
    typename SymbolType,
    uint32   SYMBOL_SIZE_T,
    bool     BIG_ENDIAN_T,
    typename OutStreamIterator,
    typename OutOffsetIterator>
struct copy_dispatch<
    ConcatenatedStringSet<
        PackedStream<OutStreamIterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T>,
        OutOffsetIterator >
    >
{
    typedef ConcatenatedStringSet<
        PackedStream<OutStreamIterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T>,
        OutOffsetIterator >
        out_string_set_type;

    // generic input set
    template <typename in_string_set_type>
    struct source_dispatch
    {
        static void enact(
            const in_string_set_type&  in_string_set,
                  out_string_set_type& out_string_set)
        {
            const uint32 BLOCKDIM = 64u;

            if (out_string_set.size() != in_string_set.size())
                throw nvbio::runtime_error( "copy() : unmatched string set sizes" );

            {
                const uint32 n_blocks = (in_string_set.size()+1 + BLOCKDIM-1)/BLOCKDIM;

                // extract the string lengths
                generic_string_lengths_kernel<BLOCKDIM> <<<n_blocks,BLOCKDIM>>>(
                    in_string_set.size(),
                    in_string_set,
                    out_string_set.offsets() );

                cudaThreadSynchronize();
            }
        #if THRUST_VERSION <= 100503
            // perform an exclusive scan on the string lengths to get the offsets
            thrust::detail::backend::cuda::exclusive_scan(
                out_string_set.offsets(),
                out_string_set.offsets() + in_string_set.size()+1,
                out_string_set.offsets(),
                0u,
                add_functor() );
        #else
            // perform an exclusive scan on the string lengths to get the offsets
            thrust::exclusive_scan(
                thrust::device,
                out_string_set.offsets(),
                out_string_set.offsets() + in_string_set.size()+1,
                out_string_set.offsets(),
                0u,
                add_functor() );
        #endif
            // extract the total string set length from the offset vector
            thrust::device_vector<uint32> d_total_length(1);

            generic_vector_copy_kernel<BLOCKDIM> <<<1,1>>> (
                1u,
                out_string_set.offsets() + in_string_set.size(),
                thrust::raw_pointer_cast( &d_total_length.front() ) );

            cudaThreadSynchronize();

            const uint32 SYMBOLS_PER_WORD = (8u*sizeof(uint32)) / SYMBOL_SIZE_T;
            const uint32 N_symbols = d_total_length[0];
            const uint32 N_words   = (N_symbols + SYMBOLS_PER_WORD-1) / SYMBOLS_PER_WORD;
            {
                const uint32 n_blocks = (N_words + BLOCKDIM-1)/BLOCKDIM;

                generic_to_packed_concat_kernel<BLOCKDIM,SYMBOL_SIZE_T,BIG_ENDIAN_T> <<<n_blocks,BLOCKDIM>>>(
                    in_string_set.size(),
                    N_words,
                    in_string_set,
                    out_string_set.base_string().stream(),
                    out_string_set.offsets() );

                cudaThreadSynchronize();
            }
        }
    };

    // concatenated input set
    template <
        typename InStringIterator,
        typename InOffsetIterator>
    struct source_dispatch<
        ConcatenatedStringSet<
            InStringIterator,
            InOffsetIterator>
        >
    {
        typedef ConcatenatedStringSet<
            InStringIterator,
            InOffsetIterator>
            in_string_set_type;

        static void enact(
            const in_string_set_type&  in_string_set,
                  out_string_set_type& out_string_set)
        {
            const uint32 BLOCKDIM = 64u;

            if (out_string_set.size() != in_string_set.size())
                throw nvbio::runtime_error( "copy() : unmatched string set sizes" );

            // extract the total string set length from the offset vector
            thrust::device_vector<uint32> d_total_length(1);

            generic_vector_copy_kernel<BLOCKDIM> <<<1,1>>> (
                1u,
                in_string_set.offsets() + in_string_set.size(),
                thrust::raw_pointer_cast( &d_total_length.front() ) );

            cudaThreadSynchronize();

            const uint32 SYMBOLS_PER_WORD = (8u*sizeof(uint32)) / SYMBOL_SIZE_T;
            const uint32 N_symbols = d_total_length[0];
            const uint32 N_words   = (N_symbols + SYMBOLS_PER_WORD-1) / SYMBOLS_PER_WORD;
            {
                const uint32 n_blocks = (N_words + BLOCKDIM-1)/BLOCKDIM;

                concat_to_packed_concat_kernel<BLOCKDIM,SYMBOL_SIZE_T,BIG_ENDIAN_T> <<<n_blocks,BLOCKDIM>>>(
                    in_string_set.size(),
                    N_symbols,
                    in_string_set,
                    out_string_set.base_string().stream(),
                    out_string_set.offsets() );

                cudaThreadSynchronize();
            }
        }
    };

    template <typename in_string_set_type>
    static void enact(
            const in_string_set_type&  in_string_set,
                  out_string_set_type& out_string_set)
    {
        return source_dispatch<in_string_set_type>::enact( in_string_set, out_string_set );
    }
};

//
// strided output set
//
template <
    typename OutStreamIterator,
    typename OutLengthIterator>
struct copy_dispatch<
    StridedStringSet<
        OutStreamIterator,
        OutLengthIterator>
    >
{
    typedef StridedStringSet<OutStreamIterator, OutLengthIterator>  out_string_set_type;

    // generic input set
    template <typename in_string_set_type>
    struct source_dispatch
    {
        static void enact(
            const in_string_set_type&  in_string_set,
                  out_string_set_type& out_string_set)
        {
            const uint32 BLOCKDIM = 64u;

            if (out_string_set.size() != in_string_set.size() ||
                out_string_set.stride() < out_string_set.size())
                throw nvbio::runtime_error( "copy() : unmatched string set sizes" );

            const uint32 n_blocks = (in_string_set.size() + BLOCKDIM-1)/BLOCKDIM;

            generic_to_strided_kernel<BLOCKDIM> <<<n_blocks,BLOCKDIM>>>(
                in_string_set.size(),
                out_string_set.stride(),
                in_string_set,
                out_string_set.base_string(),
                out_string_set.lengths() );

            cudaThreadSynchronize();
        }
    };

    // packed-concat input set
    template <
        typename InStreamIterator,
        typename SymbolType,
        uint32   SYMBOL_SIZE_T,
        bool     BIG_ENDIAN_T,
        typename InOffsetIterator>
    struct source_dispatch<
        ConcatenatedStringSet<
            PackedStream<InStreamIterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T>,
            InOffsetIterator>
        >
    {
        typedef ConcatenatedStringSet<
            PackedStream<InStreamIterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T>,
            InOffsetIterator>   in_string_set_type;

        static void enact(
            const in_string_set_type&  in_string_set,
            const out_string_set_type& out_string_set)
        {
            const uint32 BLOCKDIM = 64u;

            if (out_string_set.size() != in_string_set.size() ||
                out_string_set.stride() < out_string_set.size())
                throw nvbio::runtime_error( "copy() : unmatched string set sizes" );

            const uint32 n_blocks = (in_string_set.size() + BLOCKDIM-1)/BLOCKDIM;

            // get the base word stream of the input
            const InStreamIterator in_stream = in_string_set.base_string().stream();

            packed_concat_to_strided_kernel<
                BLOCKDIM,
                SYMBOL_SIZE_T,
                BIG_ENDIAN_T>
                <<<n_blocks,BLOCKDIM>>>(
                in_string_set.size(),
                out_string_set.stride(),
                in_stream,
                in_string_set.offsets(),
                out_string_set.base_string(),
                out_string_set.lengths() );

            cudaThreadSynchronize();
        }
    };

    // packed-sparse input set
    template <
        typename InStreamIterator,
        typename SymbolType,
        uint32   SYMBOL_SIZE_T,
        bool     BIG_ENDIAN_T,
        typename InOffsetIterator>
    struct source_dispatch<
        SparseStringSet<
            PackedStream<InStreamIterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T>,
            InOffsetIterator>
        >
    {
        typedef SparseStringSet<
            PackedStream<InStreamIterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T>,
            InOffsetIterator>   in_string_set_type;

        static void enact(
            const in_string_set_type&  in_string_set,
            const out_string_set_type& out_string_set)
        {
            const uint32 BLOCKDIM = 64u;

            if (out_string_set.size() != in_string_set.size() ||
                out_string_set.stride() < out_string_set.size())
                throw nvbio::runtime_error( "copy() : unmatched string set sizes" );

            const uint32 n_blocks = (in_string_set.size() + BLOCKDIM-1)/BLOCKDIM;

            // get the base word stream of the input
            const InStreamIterator in_stream = in_string_set.base_string().stream();

            packed_sparse_to_strided_kernel<
                BLOCKDIM,
                SYMBOL_SIZE_T,
                BIG_ENDIAN_T>
                <<<n_blocks,BLOCKDIM>>>(
                in_string_set.size(),
                out_string_set.stride(),
                in_stream,
                in_string_set.ranges(),
                out_string_set.base_string(),
                out_string_set.lengths() );

            cudaThreadSynchronize();
        }
    };

    template <typename in_string_set_type>
    static void enact(
            const in_string_set_type&  in_string_set,
                  out_string_set_type& out_string_set)
    {
        return source_dispatch<in_string_set_type>::enact( in_string_set, out_string_set );
    }
};

//
// strided output set
//
template <
    typename OutLengthIterator>
struct copy_dispatch<
    StridedStringSet<
        uint8*,
        OutLengthIterator>
    >
{
    typedef StridedStringSet<uint8*, OutLengthIterator>  out_string_set_type;

    // generic input set
    template <typename in_string_set_type>
    struct source_dispatch
    {
        static void enact(
            const in_string_set_type&  in_string_set,
                  out_string_set_type& out_string_set)
        {
            const uint32 BLOCKDIM = 64u;

            if (out_string_set.size() != in_string_set.size() ||
                out_string_set.stride() < out_string_set.size())
                throw nvbio::runtime_error( "copy() : unmatched string set sizes" );

        #if 0
            if ((out_string_set.stride() & 3) == 0)
            {
                const uint32 n_quads  = (in_string_set.size()+3) / 4u;
                const uint32 n_blocks = (n_quads + BLOCKDIM-1)/BLOCKDIM;

                generic_to_strided_uint8_kernel<BLOCKDIM> <<<n_blocks,BLOCKDIM>>>(
                    in_string_set.size(),
                    out_string_set.stride(),
                    in_string_set,
                    out_string_set.base_string(),
                    out_string_set.lengths() );
            }
            else
        #endif
            {
                const uint32 n_blocks = (in_string_set.size() + BLOCKDIM-1)/BLOCKDIM;

                generic_to_strided_kernel<BLOCKDIM> <<<n_blocks,BLOCKDIM>>>(
                    in_string_set.size(),
                    out_string_set.stride(),
                    in_string_set,
                    out_string_set.base_string(),
                    out_string_set.lengths() );
            }

            cudaThreadSynchronize();
        }
    };

    // sparse input set
    template <
        typename InStringIterator,
        typename InRangeIterator>
    struct source_dispatch<
        SparseStringSet<
            InStringIterator,
            InRangeIterator>
        >
    {
        typedef SparseStringSet<
            InStringIterator,
            InRangeIterator>
            in_string_set_type;

        static void enact(
            const in_string_set_type&  in_string_set,
                  out_string_set_type& out_string_set)
        {
            const uint32 BLOCKDIM = 64u;

            if (out_string_set.size() != in_string_set.size() ||
                out_string_set.stride() < out_string_set.size())
                throw nvbio::runtime_error( "copy() : unmatched string set sizes" );

            const uint32 n_blocks = (in_string_set.size() + BLOCKDIM-1)/BLOCKDIM;

            contig_to_strided_uint8_kernel<BLOCKDIM> <<<n_blocks,BLOCKDIM>>>(
                in_string_set.size(),
                out_string_set.stride(),
                in_string_set,
                out_string_set.base_string(),
                out_string_set.lengths() );

            cudaThreadSynchronize();
        }
    };

    // concatenated input set
    template <
        typename InStringIterator,
        typename InOffsetIterator>
    struct source_dispatch<
        ConcatenatedStringSet<
            InStringIterator,
            InOffsetIterator>
        >
    {
        typedef ConcatenatedStringSet<
            InStringIterator,
            InOffsetIterator>
            in_string_set_type;

        static void enact(
            const in_string_set_type&  in_string_set,
                  out_string_set_type& out_string_set)
        {
            const uint32 BLOCKDIM = 64u;

            if (out_string_set.size() != in_string_set.size() ||
                out_string_set.stride() < out_string_set.size())
                throw nvbio::runtime_error( "copy() : unmatched string set sizes" );

            const uint32 n_blocks = (in_string_set.size() + BLOCKDIM-1)/BLOCKDIM;

            contig_to_strided_uint8_kernel<BLOCKDIM> <<<n_blocks,BLOCKDIM>>>(
                in_string_set.size(),
                out_string_set.stride(),
                in_string_set,
                out_string_set.base_string(),
                out_string_set.lengths() );

            cudaThreadSynchronize();
        }
    };

    // strided-packed input set
    template <
        typename SymbolType,
        uint32   SYMBOL_SIZE_T,
        uint32   BIG_ENDIAN_T,
        typename InLengthIterator>
    struct source_dispatch<
        StridedPackedStringSet<
            uint32*,
            SymbolType,
            SYMBOL_SIZE_T,
            BIG_ENDIAN_T,
            InLengthIterator>
        >
    {
        typedef StridedPackedStringSet<
            uint32*,
            SymbolType,
            SYMBOL_SIZE_T,
            BIG_ENDIAN_T,
            InLengthIterator>
            in_string_set_type;

        static void enact(
            const in_string_set_type&  in_string_set,
                  out_string_set_type& out_string_set)
        {
            const uint32 BLOCKDIM = 64u;

            if (out_string_set.size() != in_string_set.size() ||
                out_string_set.stride() < out_string_set.size())
                throw nvbio::runtime_error( "copy() : unmatched string set sizes" );

            // optimize if the stride is a multiple of 4 (i.e. if we can write out full 32-bit words)
            if ((out_string_set.stride() & 3) == 0)
            {
                const uint32 n_quads  = (in_string_set.size()+3u) / 4u;
                const uint32 n_blocks = (n_quads + BLOCKDIM-1)/BLOCKDIM;

                strided_packed_to_strided_uint8_kernel<BLOCKDIM> <<<n_blocks,BLOCKDIM>>>(
                    in_string_set.size(),
                    out_string_set.stride(),
                    in_string_set,
                    out_string_set.base_string(),
                    out_string_set.lengths() );
            }
            else
            {
                const uint32 n_blocks = (in_string_set.size() + BLOCKDIM-1)/BLOCKDIM;

                generic_to_strided_kernel<BLOCKDIM> <<<n_blocks,BLOCKDIM>>>(
                    in_string_set.size(),
                    out_string_set.stride(),
                    in_string_set,
                    out_string_set.base_string(),
                    out_string_set.lengths() );
            }
            cudaThreadSynchronize();
        }
    };
    // strided-packed input set
    template <
        typename SymbolType,
        uint32   SYMBOL_SIZE_T,
        uint32   BIG_ENDIAN_T,
        typename InLengthIterator>
    struct source_dispatch<
        StridedPackedStringSet<
            const uint32*,
            SymbolType,
            SYMBOL_SIZE_T,
            BIG_ENDIAN_T,
            InLengthIterator>
        >
    {
        typedef StridedPackedStringSet<
            const uint32*,
            SymbolType,
            SYMBOL_SIZE_T,
            BIG_ENDIAN_T,
            InLengthIterator>
            in_string_set_type;

        static void enact(
            const in_string_set_type&  in_string_set,
                  out_string_set_type& out_string_set)
        {
            const uint32 BLOCKDIM = 64u;

            if (out_string_set.size() != in_string_set.size() ||
                out_string_set.stride() < out_string_set.size())
                throw nvbio::runtime_error( "copy() : unmatched string set sizes" );

            // optimize if the stride is a multiple of 4 (i.e. if we can write out full 32-bit words)
            if ((out_string_set.stride() & 3) == 0)
            {
                const uint32 n_quads  = (in_string_set.size()+3u) / 4u;
                const uint32 n_blocks = (n_quads + BLOCKDIM-1)/BLOCKDIM;

                strided_packed_to_strided_uint8_kernel<BLOCKDIM> <<<n_blocks,BLOCKDIM>>>(
                    in_string_set.size(),
                    out_string_set.stride(),
                    in_string_set,
                    out_string_set.base_string(),
                    out_string_set.lengths() );
            }
            else
            {
                const uint32 n_blocks = (in_string_set.size() + BLOCKDIM-1)/BLOCKDIM;

                generic_to_strided_kernel<BLOCKDIM> <<<n_blocks,BLOCKDIM>>>(
                    in_string_set.size(),
                    out_string_set.stride(),
                    in_string_set,
                    out_string_set.base_string(),
                    out_string_set.lengths() );
            }
            cudaThreadSynchronize();
        }
    };

    // packed-concat input set
    template <
        typename InStreamIterator,
        typename SymbolType,
        uint32   SYMBOL_SIZE_T,
        bool     BIG_ENDIAN_T,
        typename InOffsetIterator>
    struct source_dispatch<
        ConcatenatedStringSet<
            PackedStream<InStreamIterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T>,
            InOffsetIterator>
        >
    {
        typedef ConcatenatedStringSet<
            PackedStream<InStreamIterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T>,
            InOffsetIterator>   in_string_set_type;

        static void enact(
            const in_string_set_type&  in_string_set,
            const out_string_set_type& out_string_set)
        {
            const uint32 BLOCKDIM = 64u;

            if (out_string_set.size() != in_string_set.size() ||
                out_string_set.stride() < out_string_set.size())
                throw nvbio::runtime_error( "copy() : unmatched string set sizes" );

            const uint32 n_blocks = (in_string_set.size() + BLOCKDIM-1)/BLOCKDIM;

            // get the base word stream of the input
            const InStreamIterator in_stream = in_string_set.base_string().stream();

            packed_concat_to_strided_kernel<
                BLOCKDIM,
                SYMBOL_SIZE_T,
                BIG_ENDIAN_T>
                <<<n_blocks,BLOCKDIM>>>(
                in_string_set.size(),
                out_string_set.stride(),
                in_stream,
                in_string_set.offsets(),
                out_string_set.base_string(),
                out_string_set.lengths() );

            cudaThreadSynchronize();
        }
    };

    // packed-sparse input set
    template <
        typename InStreamIterator,
        typename SymbolType,
        uint32   SYMBOL_SIZE_T,
        bool     BIG_ENDIAN_T,
        typename InOffsetIterator>
    struct source_dispatch<
        SparseStringSet<
            PackedStream<InStreamIterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T>,
            InOffsetIterator>
        >
    {
        typedef SparseStringSet<
            PackedStream<InStreamIterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T>,
            InOffsetIterator>   in_string_set_type;

        static void enact(
            const in_string_set_type&  in_string_set,
            const out_string_set_type& out_string_set)
        {
            const uint32 BLOCKDIM = 64u;

            if (out_string_set.size() != in_string_set.size() ||
                out_string_set.stride() < out_string_set.size())
                throw nvbio::runtime_error( "copy() : unmatched string set sizes" );

            const uint32 n_blocks = (in_string_set.size() + BLOCKDIM-1)/BLOCKDIM;

            // get the base word stream of the input
            const InStreamIterator in_stream = in_string_set.base_string().stream();

            packed_sparse_to_strided_kernel<
                BLOCKDIM,
                SYMBOL_SIZE_T,
                BIG_ENDIAN_T>
                <<<n_blocks,BLOCKDIM>>>(
                in_string_set.size(),
                out_string_set.stride(),
                in_stream,
                in_string_set.ranges(),
                out_string_set.base_string(),
                out_string_set.lengths() );

            cudaThreadSynchronize();
        }
    };

    template <typename in_string_set_type>
    static void enact(
            const in_string_set_type&  in_string_set,
                  out_string_set_type& out_string_set)
    {
        return source_dispatch<in_string_set_type>::enact( in_string_set, out_string_set );
    }
};


//
// strided-packed output string sets.
//
template <
    typename SymbolType,
    uint32   SYMBOL_SIZE_T,
    bool     BIG_ENDIAN_T,
    typename OutStreamIterator,
    typename OutLengthIterator>
struct copy_dispatch<
    StridedPackedStringSet<
        OutStreamIterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T,
        OutLengthIterator>
    >
{
    typedef StridedPackedStringSet<
        OutStreamIterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T,
        OutLengthIterator>                                          out_string_set_type;

    // generic input set
    template <typename in_string_set_type>
    struct source_dispatch
    {
        static void enact(
            const in_string_set_type&  in_string_set,
            const out_string_set_type& out_string_set)
        {
            const uint32 BLOCKDIM = 64u;

            if (out_string_set.size() != in_string_set.size() ||
                out_string_set.stride() < out_string_set.size())
                throw nvbio::runtime_error( "copy() : unmatched string set sizes" );

            const uint32 n_blocks = (in_string_set.size() + BLOCKDIM-1)/BLOCKDIM;

            generic_to_strided_packed_kernel<
                BLOCKDIM,
                SYMBOL_SIZE_T,
                BIG_ENDIAN_T>
                <<<n_blocks,BLOCKDIM>>>(
                in_string_set.size(),
                out_string_set.stride(),
                in_string_set,
                out_string_set.base_stream(),
                out_string_set.lengths() );

            cudaThreadSynchronize();
        }
    };

    // concatenated input set
    template <
        typename InStreamIterator,
        typename InOffsetIterator>
    struct source_dispatch<
        ConcatenatedStringSet<
            InStreamIterator,
            InOffsetIterator>
        >
    {
        typedef ConcatenatedStringSet<
            InStreamIterator,
            InOffsetIterator>   in_string_set_type;

        static void enact(
            const in_string_set_type&  in_string_set,
            const out_string_set_type& out_string_set)
        {
            const uint32 BLOCKDIM = 128u;

            if (out_string_set.size() != in_string_set.size() ||
                out_string_set.stride() < out_string_set.size())
                throw nvbio::runtime_error( "copy() : unmatched string set sizes" );

            const uint32 n_blocks = (in_string_set.size() + BLOCKDIM-1)/BLOCKDIM;

            contig_to_strided_packed_kernel<
                BLOCKDIM,
                SYMBOL_SIZE_T,
                BIG_ENDIAN_T>
                <<<n_blocks,BLOCKDIM>>>(
                in_string_set.size(),
                out_string_set.stride(),
                in_string_set,
                out_string_set.base_stream(),
                out_string_set.lengths() );

            cudaThreadSynchronize();
        }
    };

    // sparse input set
    template <
        typename InStreamIterator,
        typename InOffsetIterator>
    struct source_dispatch<
        SparseStringSet<
            InStreamIterator,
            InOffsetIterator>
        >
    {
        typedef SparseStringSet<
            InStreamIterator,
            InOffsetIterator>   in_string_set_type;

        static void enact(
            const in_string_set_type&  in_string_set,
            const out_string_set_type& out_string_set)
        {
            const uint32 BLOCKDIM = 64u;

            if (out_string_set.size() != in_string_set.size() ||
                out_string_set.stride() < out_string_set.size())
                throw nvbio::runtime_error( "copy() : unmatched string set sizes" );

            const uint32 n_blocks = (in_string_set.size() + BLOCKDIM-1)/BLOCKDIM;

            contig_to_strided_packed_kernel<
                BLOCKDIM,
                SYMBOL_SIZE_T,
                BIG_ENDIAN_T>
                <<<n_blocks,BLOCKDIM>>>(
                in_string_set.size(),
                out_string_set.stride(),
                in_string_set,
                out_string_set.base_stream(),
                out_string_set.lengths() );

            cudaThreadSynchronize();
        }
    };

    // packed-concatenated input set
    template <
        typename InStreamIterator,
        typename InOffsetIterator>
    struct source_dispatch<
        ConcatenatedStringSet<
            PackedStream<InStreamIterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T>,
            InOffsetIterator>
        >
    {
        typedef ConcatenatedStringSet<
            PackedStream<InStreamIterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T>,
            InOffsetIterator>   in_string_set_type;

        static void enact(
            const in_string_set_type&  in_string_set,
            const out_string_set_type& out_string_set)
        {
            const uint32 BLOCKDIM = 64;

            if (out_string_set.size() != in_string_set.size() ||
                out_string_set.stride() < out_string_set.size())
                throw nvbio::runtime_error( "copy() : unmatched string set sizes" );

            const uint32 n_blocks = (in_string_set.size() + BLOCKDIM-1)/BLOCKDIM;

            // get the base word stream of the input
            const InStreamIterator in_stream = in_string_set.base_string().stream();

            packed_concatenated_to_strided_packed_kernel<
                BLOCKDIM,
                SYMBOL_SIZE_T,
                BIG_ENDIAN_T>
                <<<n_blocks,BLOCKDIM>>>(
                in_string_set.size(),
                out_string_set.stride(),
                in_stream,
                in_string_set.offsets(),
                out_string_set.base_stream(),
                out_string_set.lengths() );

            cudaThreadSynchronize();
        }
    };

    // packed-sparse input set
    template <
        typename InStreamIterator,
        typename InOffsetIterator>
    struct source_dispatch<
        SparseStringSet<
            PackedStream<InStreamIterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T>,
            InOffsetIterator>
        >
    {
        typedef SparseStringSet<
            PackedStream<InStreamIterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T>,
            InOffsetIterator>   in_string_set_type;

        static void enact(
            const in_string_set_type&  in_string_set,
            const out_string_set_type& out_string_set)
        {
            const uint32 BLOCKDIM = 64u;

            if (out_string_set.size() != in_string_set.size() ||
                out_string_set.stride() < out_string_set.size())
                throw nvbio::runtime_error( "copy() : unmatched string set sizes" );

            const uint32 n_blocks = (in_string_set.size() + BLOCKDIM-1)/BLOCKDIM;

            // get the base word stream of the input
            const InStreamIterator in_stream = in_string_set.base_string().stream();

            packed_sparse_to_strided_packed_kernel<
                BLOCKDIM,
                SYMBOL_SIZE_T,
                BIG_ENDIAN_T>
                <<<n_blocks,BLOCKDIM>>>(
                in_string_set.size(),
                out_string_set.stride(),
                in_stream,
                in_string_set.ranges(),
                out_string_set.base_stream(),
                out_string_set.lengths() );

            cudaThreadSynchronize();
        }
    };

    template <typename in_string_set_type>
    static void enact(
            const in_string_set_type&  in_string_set,
                  out_string_set_type& out_string_set)
    {
        return source_dispatch<in_string_set_type>::enact( in_string_set, out_string_set );
    }
};

// copy a generic string set into a concatenated one
//
// \param in_string_set        input string set
// \param out_string_set       output string set
//
template <
    typename InStringSet,
    typename StringIterator,
    typename OffsetIterator>
void copy(
    const InStringSet&  in_string_set,
          ConcatenatedStringSet<StringIterator,OffsetIterator>& out_string_set)
{
    typedef ConcatenatedStringSet<StringIterator,OffsetIterator> OutStringSet;

    copy_dispatch<OutStringSet>::enact( in_string_set, out_string_set );
}

// copy a generic string set into a strided one
//
// \param in_string_set        input string set
// \param out_string_set       output string set
//
template <
    typename InStringSet,
    typename StringIterator,
    typename LengthIterator>
void copy(
    const InStringSet&  in_string_set,
          StridedStringSet<StringIterator,LengthIterator>& out_string_set)
{
    typedef StridedStringSet<StringIterator,LengthIterator> OutStringSet;

    copy_dispatch<OutStringSet>::enact( in_string_set, out_string_set );
}

// copy a generic string set into a strided-packed one
//
// \param in_string_set        input string set
// \param out_string_set       output string set
//
template <
    typename InStringSet,
    typename StreamIterator,
    typename SymbolType,
    uint32   SYMBOL_SIZE_T,
    bool     BIG_ENDIAN_T,
    typename LengthIterator>
void copy(
    const InStringSet&  in_string_set,
          StridedPackedStringSet<StreamIterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T,LengthIterator>& out_string_set)
{
    typedef StridedPackedStringSet<StreamIterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T,LengthIterator> OutStringSet;

    copy_dispatch<OutStringSet>::enact( in_string_set, out_string_set );
}

} // namespace cuda

#endif // defined(__CUDACC__)

template <typename out_string_set_type>
struct copy_dispatch
{
    // generic input set
    template <typename in_string_set_type>
    struct source_dispatch
    {
        static void enact(
            const in_string_set_type&  in_string_set,
            const out_string_set_type& out_string_set)
        {
            if (out_string_set.size() != in_string_set.size() ||
                out_string_set.stride() < out_string_set.size())
                throw nvbio::runtime_error( "copy() : unmatched string set sizes" );

            const uint32 n_strings = in_string_set.size();

        #if 1
            const uint32 BLOCK_SIZE = 16;
            for (uint32 i_block = 0; i_block < n_strings; i_block += BLOCK_SIZE)
            {
                const uint32 i_block_end = std::min( i_block + BLOCK_SIZE, n_strings );

                uint32 max_len = 0;
                for (uint32 i = i_block; i < i_block_end; ++i)
                    max_len = std::max( max_len, in_string_set[i].length() );

                for (uint32 j_block = 0; j_block < max_len; j_block += BLOCK_SIZE)
                {
                    for (uint32 i = i_block; i < i_block_end; ++i)
                    {
                        typename  in_string_set_type::string_type  in_string =  in_string_set[i];
                        typename out_string_set_type::string_type out_string = out_string_set[i];

                        const uint32 m = in_string.length();
                        const uint32 j_block_end = std::min( j_block + BLOCK_SIZE, m );

                        for (uint32 j = j_block; j < j_block_end; ++j)
                            out_string[j] = in_string[j];
                    }
                }
            }
        #else
            for (uint32 i = 0; i < n_strings; ++i)
            {
                typename  in_string_set_type::string_type  in_string =  in_string_set[i];
                typename out_string_set_type::string_type out_string = out_string_set[i];

                const uint32 m = in_string.length();
                for (uint32 j = 0; j < m; ++j)
                    out_string[j] = in_string[j];
            }
        #endif
        }
    };

    template <typename in_string_set_type>
    static void enact(
            const in_string_set_type&  in_string_set,
                  out_string_set_type& out_string_set)
    {
        return source_dispatch<in_string_set_type>::enact( in_string_set, out_string_set );
    }
};

// copy a generic string set into a concatenated one
//
// \param in_string_set        input string set
// \param out_string_set       output string set
//
template <
    typename InStringSet,
    typename StringIterator,
    typename OffsetIterator>
void copy(
    const InStringSet&  in_string_set,
          ConcatenatedStringSet<StringIterator,OffsetIterator>& out_string_set)
{
    typedef ConcatenatedStringSet<StringIterator,OffsetIterator> OutStringSet;

    copy_dispatch<OutStringSet>::enact( in_string_set, out_string_set );
}

// copy a generic string set into a strided one
//
// \param in_string_set        input string set
// \param out_string_set       output string set
//
template <
    typename InStringSet,
    typename StringIterator,
    typename LengthIterator>
void copy(
    const InStringSet&  in_string_set,
          StridedStringSet<StringIterator,LengthIterator>& out_string_set)
{
    typedef StridedStringSet<StringIterator,LengthIterator> OutStringSet;

    copy_dispatch<OutStringSet>::enact( in_string_set, out_string_set );
}

// copy a generic string set into a strided-packed one
//
// \param in_string_set        input string set
// \param out_string_set       output string set
//
template <
    typename InStringSet,
    typename StreamIterator,
    typename SymbolType,
    uint32   SYMBOL_SIZE_T,
    bool     BIG_ENDIAN_T,
    typename LengthIterator>
void copy(
    const InStringSet&  in_string_set,
          StridedPackedStringSet<StreamIterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T,LengthIterator>& out_string_set)
{
    typedef StridedPackedStringSet<StreamIterator,SymbolType,SYMBOL_SIZE_T,BIG_ENDIAN_T,LengthIterator> OutStringSet;

    copy_dispatch<OutStringSet>::enact( in_string_set, out_string_set );
}

} // namespace nvbio
