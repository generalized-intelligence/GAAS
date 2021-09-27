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

#include <cub/cub.cuh>
#include <mgpuhost.cuh>
#include <moderngpu.cuh>
#include <nvbio/strings/string_set.h>
#include <nvbio/basic/thrust_view.h>
#include <nvbio/basic/cuda/sort.h>
#include <nvbio/basic/cuda/timer.h>
#include <nvbio/basic/cuda/ldg.h>
#include <nvbio/basic/cuda/primitives.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/adjacent_difference.h>
#include <thrust/binary_search.h>
#include <thrust/iterator/constant_iterator.h>

#if defined(PLATFORM_X86)
#include <emmintrin.h>                              // SSE intrinsics
#endif

namespace nvbio {
namespace priv {

template <uint32 BITS> struct word_selector     {};
template <>            struct word_selector<4>  { typedef uint8 type; };
template <>            struct word_selector<6>  { typedef uint8 type; };
template <>            struct word_selector<8>  { typedef uint8 type; };
template <>            struct word_selector<10> { typedef uint8 type; };
template <>            struct word_selector<12> { typedef uint16 type; };
template <>            struct word_selector<14> { typedef uint16 type; };
template <>            struct word_selector<16> { typedef uint16 type; };
template <>            struct word_selector<18> { typedef uint16 type; };
template <>            struct word_selector<20> { typedef uint32 type; };
template <>            struct word_selector<22> { typedef uint32 type; };
template <>            struct word_selector<24> { typedef uint32 type; };
template <>            struct word_selector<26> { typedef uint32 type; };
template <>            struct word_selector<28> { typedef uint32 type; };
template <>            struct word_selector<30> { typedef uint32 type; };
template <>            struct word_selector<32> { typedef uint32 type; };
template <>            struct word_selector<48> { typedef uint64 type; };
template <>            struct word_selector<64> { typedef uint64 type; };

typedef ConcatenatedStringSet<
    PackedStream<uint32*,uint8,2u,false,uint64>,
    uint64*>                                                string_set_2bit;

typedef ConcatenatedStringSet<
    PackedStream<uint32*,uint8,2u,false,uint64>,
    uint64*>                                                string_set_4bit;

typedef ConcatenatedStringSet<
    PackedStream<uint32*,uint8,8u,false,uint64>,
    uint64*>                                                string_set_8bit;

typedef ConcatenatedStringSet<
    PackedStream<uint32*,uint8,2u,true,uint64>,
    uint64*>                                                string_set_2bit_be;

typedef ConcatenatedStringSet<
    PackedStream<uint64*,uint8,2u,true,uint64>,
    uint64*>                                                string_set_2bit_u64_be;

typedef PackedStream<uint32*,uint8,2u,false,uint64> string_2bit_le;
typedef PackedStream<uint32*,uint8,4u,false,uint64> string_4bit_le;
typedef PackedStream<uint32*,uint8,8u,false,uint64> string_8bit_le;
typedef PackedStream<uint32*,uint8,2u,true,uint64>  string_2bit_be;
typedef PackedStream<uint32*,uint8,4u,true,uint64>  string_4bit_be;
typedef PackedStream<uint32*,uint8,8u,true,uint64>  string_8bit_be;

void extract_radices(
    const priv::string_set_2bit_be  string_set,
    const uint32                    n_suffixes,
    const uint32                    word_begin,
    const uint32                    word_end,
    const uint32                    word_bits,
    const uint2*                    suffixes,
          uint32*                   radices,
          uint8*                    symbols = NULL);

void extract_radices(
    const priv::string_set_2bit_u64_be  string_set,
    const uint32                        n_suffixes,
    const uint32                        word_begin,
    const uint32                        word_end,
    const uint32                        word_bits,
    const uint2*                        suffixes,
          uint64*                       radices,
          uint8*                        symbols = NULL);

// make sure a given buffer is big enough
//
template <typename VectorType>
void alloc_storage(VectorType& vec, const uint64 size)
{
    if (vec.size() < size)
    {
        try
        {
            vec.clear();
            vec.resize( size );
        }
        catch (...)
        {
            log_error(stderr,"alloc_storage() : allocation failed!\n");
            throw;
        }
    }
}

/// set the last n bits to 0
///
template <typename storage_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
storage_type clearmask(const uint32 n) { return ~((storage_type(1u) << n)-1u); }

/// A functor to cast from one type into another
///
struct in_range_functor
{
    typedef uint32 argument_type;
    typedef bool   result_type;

    /// constructor
    ///
    in_range_functor(const uint32 _begin, const uint32 _end) : begin(_begin), end(_end) {}

    /// return true if i is in the range
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    bool operator() (const uint32 i) const { return i >= begin && i < end; }

    const uint32 begin, end;
};

/// A functor subtracting the second element of a pair from the first
///
struct minus_one
{
    typedef uint32 argument_type;
    typedef uint32 result_type;

    /// return the length of the i-th string
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 operator() (const uint32 i) const { return i - 1; }
};

/// A functor adding the given constant to all intergers
///
struct offset_functor
{
    typedef uint32 argument_type;
    typedef uint32 result_type;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    offset_functor(const uint32 _offset) : offset(_offset) {}

    /// return the length of the i-th string
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 operator() (const uint32 i) const { return i + offset; }

    const uint32 offset;
};

/// A functor dividing all integers by the given constant
///
struct add_divide_functor
{
    typedef uint32 argument_type;
    typedef uint32 result_type;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    add_divide_functor(const uint32 _a, const uint32 _k) : a(_a), k(_k) {}

    /// return the length of the i-th string
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 operator() (const uint32 i) const { return (i + a) / k; }

    const uint32 a;
    const uint32 k;
};

/// A functor fetching the length of the i-th string in a set
///
template <typename string_set_type>
struct length_functor
{
    typedef uint32 argument_type;
    typedef uint32 result_type;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    length_functor(const string_set_type _string_set, const bool _extended) : string_set(_string_set), extended(_extended) {}

    /// return the length of the i-th string
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 operator() (const uint32 i) const
    {
        return string_set[i].length() + (extended ? 1u : 0u);
    }

    string_set_type string_set;
    bool            extended;
};

/// A functor adding the given constant to the string id of a suffix
///
struct suffix_offset_functor
{
    typedef uint2 argument_type;
    typedef uint2 result_type;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    suffix_offset_functor(const uint32 _offset) : offset(_offset) {}

    /// return the length of the i-th string
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint2 operator() (const uint2 suffix) const { return make_uint2( suffix.x, suffix.y + offset ); }

    const uint32 offset;
};

/// A functor returning the given component of a suffix
///
enum SuffixComponent
{
    SUFFIX_ID = 0,
    STRING_ID = 1
};

template <SuffixComponent COMP>
struct suffix_component_functor
{
    typedef uint2  argument_type;
    typedef uint32 result_type;

    /// return the length of the i-th string
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 operator() (const uint2 suffix) const { return COMP == STRING_ID ? suffix.y : suffix.x; }
};

template <uint32 WORD_BITS, uint32 DOLLAR_BITS, uint32 SYMBOL_SIZE, typename string_type, typename index_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32 extract_word_generic(
    const string_type   string,
    const index_type    string_len,
    const index_type    suffix_idx,
    const uint32        w)
{
    const uint32 SYMBOLS_PER_WORD = uint32(WORD_BITS - DOLLAR_BITS)/SYMBOL_SIZE;
    const uint32 SYMBOL_OFFSET    = uint32(WORD_BITS) - SYMBOL_SIZE;

    uint32 word = 0u;
    for (uint32 j = 0; j < SYMBOLS_PER_WORD; ++j)
    {
        const index_type jj = suffix_idx + w*SYMBOLS_PER_WORD + j;
        const uint32 c = jj < string_len ? string[jj] : 0u;
        word |= (c << (SYMBOL_OFFSET - j*SYMBOL_SIZE));
    }

    if (DOLLAR_BITS)
    {
        // encode the dollar's position in the least significant bits of the word
        const uint32 dollar_offset = 
            string_len <= suffix_idx + w*SYMBOLS_PER_WORD + SYMBOLS_PER_WORD ?  // is there a dollar sign?
                      (string_len < suffix_idx + w*SYMBOLS_PER_WORD) ? 0u  :
                uint32(string_len - suffix_idx - w*SYMBOLS_PER_WORD)         :
            (1u << DOLLAR_BITS)-1u;                                             // no dollar sign in this word

        return word | dollar_offset;
    }
    else
        return word;
}

/// return how many symbols are encoded per word
///
template <uint32 SYMBOL_SIZE, uint32 WORD_BITS, uint32 DOLLAR_BITS>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32 symbols_per_word()
{
    const uint32 SYMBOLS_PER_WORD = (WORD_BITS - DOLLAR_BITS)/SYMBOL_SIZE;
    return SYMBOLS_PER_WORD;
}

template <uint32 WORD_BITS, uint32 DOLLAR_BITS, uint32 SYMBOL_SIZE, typename storage_type, typename index_type, typename sufindex_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
typename std::iterator_traits<storage_type>::value_type extract_word_packed(
    const storage_type      base_words,
    const index_type        string_len,
    const index_type        string_off,
    const sufindex_type     suffix_idx,
    const uint32            w)
{
    typedef typename std::iterator_traits<storage_type>::value_type word_type;

    const uint32 STORAGE_BITS     = uint32( 8u * sizeof(word_type) );
    const uint32 STORAGE_SYMBOLS  = STORAGE_BITS / SYMBOL_SIZE;
    const uint32 SYMBOLS_PER_WORD = uint32(WORD_BITS - DOLLAR_BITS)/SYMBOL_SIZE;
    //const uint32 SYMBOL_OFFSET    = uint32(WORD_BITS) - SYMBOL_SIZE;

    const sufindex_type suffix_off = suffix_idx + w*SYMBOLS_PER_WORD;   // suffix offset

    // do we have any symbols to encode?
    if (suffix_off >= string_len)
        return 0u;

    const index_type range_len  = string_len - suffix_off;              // partial suffix length
    const index_type range_off  = string_off + suffix_off;              // partial suffix offset

    const uint32 n_symbols = (uint32)nvbio::min( range_len, index_type(SYMBOLS_PER_WORD) );     // symbols to pack

    //
    // As SYMBOLS_PER_WORD is less than 32, we know that the symbols we are looking for
    // will span at most 2 32-bit words.
    // Of the n_symbols we want to read, there might be n1 in the first word, and n2 in the
    // second word.
    // As the beginning of our symbol stream (range_off) might stride the 32-bit word boundary,
    // the highest m1 = range_off % 16 symbols of the first word might have to be discarded,
    // and we'll find our n1 symbols in the following position:
    //
    // |-------------------------------------------|
    // |* * * * * *| x x x x x x x x x | * * * * * |
    // |-------------------------------------------|
    // |    m1     |         n1        |     r1    |
    //
    // What we do is shifting the n1 symbols to the top of the 32-bit word (i.e. m1 to the left).
    // Clearing the remaining symbols is only needed if n1 == n_symbols; if n1 < n_symbols, r1 will
    // be necessarily zero.
    //
    // At this point, we might have n2 more symbols to read in the highest bits of the second word:
    //
    // |-------------------------------------------|
    // | y y y y y y y y | * * * * * * * * * * * * |
    // |-------------------------------------------|
    // |        n2       |            r2           |
    //
    // which we need to shift right by (n1*SYMBOL_SIZE) bits.
    // At the very end, we'll shift everything right by (32 - WORD_BITS) bits in order to have
    // our output tightly packed in the lowest WORD_BITS:
    //
    // 32       WORD_BITS            DOLLAR_BITS   0
    // |-----------|-----------------------|-------|
    // | * * * * * | x x x x | y y y | 0 0 | $ $ $ |  // notice the possible presence of 0's before 
    // |-------------------------------------------|  // the $ sign: these are bits that need to be
    // |           |    n1   |   n2  |     |       |  // cleared if the suffix is short
    //

    const uint32 k1 = uint32( range_off/STORAGE_SYMBOLS );              // index of the first word

    const uint32 m1 = range_off & (STORAGE_SYMBOLS-1);                  // offset in the word
    const uint32 r1 = STORAGE_SYMBOLS - m1;                             // symbols to read
    const word_type word1 = (base_words[ k1 ] << (m1*SYMBOL_SIZE));     // fetch the first word, shifted left

    word_type word = word1;

    if (n_symbols > r1) // do we need to read another word?
    {
        const word_type word2 = base_words[ k1+1u ];                    // fetch the second word
        word |= word2 >> (r1*SYMBOL_SIZE);                              // shift by n1 symbols to the right
    }

    word >>= (STORAGE_BITS - WORD_BITS);                                 // align the top to WORD_BITS

    // clear every symbol we don't need among the word's LSD
    word &= clearmask<word_type>( WORD_BITS - n_symbols*SYMBOL_SIZE );

    if (DOLLAR_BITS)
    {
        // encode the dollar's position in the least significant bits of the word
        const word_type dollar_offset = 
            range_len <= SYMBOLS_PER_WORD ?  // is there a dollar sign?
                range_len                 :
                (1u << DOLLAR_BITS)-1u;      // no dollar sign in this word

        return word | dollar_offset;
    }
    else
        return word;
}

template <uint32 WORD_BITS, uint32 DOLLAR_BITS, uint32 SYMBOL_SIZE, typename storage_type, typename index_type, typename sufindex_type, typename output_iterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void extract_word_packed(
    const storage_type      base_words,
    const index_type        string_len,
    const index_type        string_off,
    const sufindex_type     suffix_idx,
    const uint32            word_begin,
    const uint32            word_end,
    output_iterator         words)
{
    typedef typename std::iterator_traits<storage_type>::value_type word_type;

    const uint32 STORAGE_BITS     = uint32( 8u * sizeof(word_type) );
    const uint32 STORAGE_SYMBOLS  = STORAGE_BITS / SYMBOL_SIZE;
    const uint32 SYMBOLS_PER_WORD = uint32(WORD_BITS - DOLLAR_BITS)/SYMBOL_SIZE;
    //const uint32 SYMBOL_OFFSET    = uint32(WORD_BITS) - SYMBOL_SIZE;

    sufindex_type suffix_off = suffix_idx + word_begin*SYMBOLS_PER_WORD;   // suffix offset

    index_type range_len  = string_len - suffix_off;              // partial suffix length
    index_type range_off  = string_off + suffix_off;              // partial suffix offset

    const uint32 cache_begin = uint32( range_off / STORAGE_SYMBOLS );

  #if defined(PLATFORM_X86) && !defined(NVBIO_DEVICE_COMPILATION)
    // use SSE to load all the words we need in a small cache
    const uint32 SSE_WORDS = 16u / sizeof( word_type );
    const uint32 cache_end = uint32( (range_off + (word_end - word_begin)*SYMBOLS_PER_WORD) / STORAGE_SYMBOLS );

    __m128i sse_cache[8];
    for (uint32 w = cache_begin; w < cache_end; w += SSE_WORDS)
        sse_cache[ (w - cache_begin)/SSE_WORDS ] = _mm_loadu_si128( (const __m128i*)(base_words + w) );

    const word_type* cached_words = (const word_type*)sse_cache;
  #elif 0
    const_cached_iterator<storage_type> cached_words( base_words + cache_begin );
  #else
    const storage_type cached_words = base_words + cache_begin;
  #endif

    for (uint32 w = word_begin; w < word_end; ++w)
    {
        // do we have any symbols to encode?
        if (suffix_off >= string_len)
        {
            words[w - word_begin] = 0u;
            continue;
        }

        const uint32 n_symbols = (uint32)nvbio::min( range_len, index_type(SYMBOLS_PER_WORD) );     // symbols to pack

        //
        // As SYMBOLS_PER_WORD is less than 32, we know that the symbols we are looking for
        // will span at most 2 32-bit words.
        // Of the n_symbols we want to read, there might be n1 in the first word, and n2 in the
        // second word.
        // As the beginning of our symbol stream (range_off) might stride the 32-bit word boundary,
        // the highest m1 = range_off % 16 symbols of the first word might have to be discarded,
        // and we'll find our n1 symbols in the following position:
        //
        // |-------------------------------------------|
        // |* * * * * *| x x x x x x x x x | * * * * * |
        // |-------------------------------------------|
        // |    m1     |         n1        |     r1    |
        //
        // What we do is shifting the n1 symbols to the top of the 32-bit word (i.e. m1 to the left).
        // Clearing the remaining symbols is only needed if n1 == n_symbols; if n1 < n_symbols, r1 will
        // be necessarily zero.
        //
        // At this point, we might have n2 more symbols to read in the highest bits of the second word:
        //
        // |-------------------------------------------|
        // | y y y y y y y y | * * * * * * * * * * * * |
        // |-------------------------------------------|
        // |        n2       |            r2           |
        //
        // which we need to shift right by (n1*SYMBOL_SIZE) bits.
        // At the very end, we'll shift everything right by (32 - WORD_BITS) bits in order to have
        // our output tightly packed in the lowest WORD_BITS:
        //
        // 32       WORD_BITS            DOLLAR_BITS   0
        // |-----------|-----------------------|-------|
        // | * * * * * | x x x x | y y y | 0 0 | $ $ $ |  // notice the possible presence of 0's before 
        // |-------------------------------------------|  // the $ sign: these are bits that need to be
        // |           |    n1   |   n2  |     |       |  // cleared if the suffix is short
        //
        const uint32 k1 = uint32( range_off/STORAGE_SYMBOLS ) - cache_begin; // index of the first word

        const uint32 m1 = range_off & (STORAGE_SYMBOLS-1);                  // offset in the word
        const uint32 r1 = STORAGE_SYMBOLS - m1;                             // symbols left in the word
        const word_type word1 = (cached_words[ k1 ] << (m1*SYMBOL_SIZE));   // fetch the first word, shifted left

        word_type word = word1;

        if (n_symbols > r1) // do we need to read another word?
        {
            const word_type word2 = cached_words[ k1+1u ];                  // fetch the second word
            word |= word2 >> (r1*SYMBOL_SIZE);                              // shift by n1 symbols to the right
        }

        word >>= (STORAGE_BITS - WORD_BITS);                                // align the top to WORD_BITS

        // clear every symbol we don't need among the word's LSD
        word &= clearmask<word_type>( WORD_BITS - n_symbols*SYMBOL_SIZE );

        if (DOLLAR_BITS)
        {
            // encode the dollar's position in the least significant bits of the word
            const word_type dollar_offset = 
                range_len <= SYMBOLS_PER_WORD ?  // is there a dollar sign?
                    range_len                 :
                    (1u << DOLLAR_BITS)-1u;      // no dollar sign in this word

            word |= dollar_offset;
        }

        // write the word out
        words[ w - word_begin ] = word;

        suffix_off += SYMBOLS_PER_WORD;
        range_len  -= SYMBOLS_PER_WORD;
        range_off  += SYMBOLS_PER_WORD;
    }
}

/// A functor to localize suffixes, making the conversion: global-suffix-id -> (string-id,suffix-id)
///
struct localize_suffix_functor
{
    typedef uint32  argument_type;
    typedef uint2   result_type;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    localize_suffix_functor(const uint32* _cum_lengths, const uint32* _string_ids, const uint32 _string_offset = 0u) :
        cum_lengths(_cum_lengths),
        string_ids(_string_ids),
        string_offset( _string_offset ) {}

    /// return the localized suffix
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    result_type operator() (const uint32 global_suffix_idx) const
    {
        const uint32 string_idx = string_ids[ global_suffix_idx ];
        const uint32 suffix_idx = global_suffix_idx - (string_idx ? cum_lengths[ string_idx-1u ] : 0u);

        return make_uint2( suffix_idx, string_offset + string_idx );
    }

    const uint32*   cum_lengths;
    const uint32*   string_ids;
    const uint32    string_offset;
};

/// A functor fetching the w'th word worth of 2-bit symbols from the i-th string in a set
///
template <uint32 SYMBOL_SIZE, uint32 WORD_BITS, uint32 DOLLAR_BITS, typename string_set_type, typename word_type>
struct local_set_suffix_word_functor
{
    typedef uint2       argument_type;
    typedef word_type   result_type;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    local_set_suffix_word_functor(const string_set_type _string_set, const uint32 _w) :
        string_set(_string_set),
        w(_w) {}

    /// return the w'th word of the i-th string
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    result_type operator() (const uint2 local_suffix_idx) const
    {
        typedef typename string_set_type::string_type string_type;

        const uint32 string_idx = local_suffix_idx.y;
        const uint32 suffix_idx = local_suffix_idx.x;

        const string_type string = string_set[string_idx];
        const uint32 string_len  = string.length();

        return result_type( extract_word_generic<WORD_BITS,DOLLAR_BITS,SYMBOL_SIZE>(
            string,
            string_len,
            suffix_idx,
            w ) );
    }

    string_set_type string_set;
    uint32          w;
};

/// A functor fetching the w'th word worth of 2-bit symbols from the given (string,suffix) in a set
///
template <uint32 SYMBOL_SIZE, uint32 WORD_BITS, uint32 DOLLAR_BITS, typename storage_type, typename word_type, typename offsets_iterator>
struct local_set_suffix_word_functor<
    SYMBOL_SIZE, WORD_BITS, DOLLAR_BITS,
    ConcatenatedStringSet<
        PackedStream<storage_type,uint8,SYMBOL_SIZE,true,typename std::iterator_traits<offsets_iterator>::value_type>,
        offsets_iterator>,
    word_type>
{
    typedef typename std::iterator_traits<offsets_iterator>::value_type index_type;
    typedef ConcatenatedStringSet<
        PackedStream<storage_type,uint8,SYMBOL_SIZE,true,index_type>,
        offsets_iterator>        string_set_type;

    typedef uint2               argument_type;
    typedef word_type           result_type;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    local_set_suffix_word_functor(const string_set_type _string_set, const uint32 _w) :
        string_set(_string_set),
        w(_w) {}
 
    /// return the w'th word of the i-th string
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    result_type operator() (const uint2 local_suffix_idx) const
    {
        typedef typename string_set_type::string_type string_type;

        const uint32 string_idx = local_suffix_idx.y;
        const uint32 suffix_idx = local_suffix_idx.x;

        const index_type string_off = string_set.offsets()[ string_idx ];
        const index_type string_end = string_set.offsets()[ string_idx+1u ];
        const index_type string_len = uint32( string_end - string_off );

        const storage_type base_words = string_set.base_string().stream();

        return result_type( extract_word_packed<WORD_BITS,DOLLAR_BITS,SYMBOL_SIZE>(
            base_words,
            string_len,
            string_off,
            suffix_idx,
            w ) );
    }

    string_set_type string_set;
    uint32          w;
};

/// A functor fetching the w'th word worth of 2-bit symbols from the i-th suffix in a set
///
template <uint32 SYMBOL_SIZE, uint32 WORD_BITS, uint32 DOLLAR_BITS, typename string_set_type, typename word_type>
struct global_set_suffix_word_functor
{
    typedef uint32      argument_type;
    typedef word_type   result_type;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    global_set_suffix_word_functor(const string_set_type _string_set, const uint32* _cum_lengths, const uint32* _string_ids, const uint32 _w) :
        word_functor( _string_set, _w ),
        localizer( _cum_lengths, _string_ids ) {}

    /// return the w'th word of the i-th string
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    result_type operator() (const uint32 global_suffix_idx) const
    {
        return word_functor( localizer( global_suffix_idx ) );
    }

    local_set_suffix_word_functor<SYMBOL_SIZE,WORD_BITS,DOLLAR_BITS,string_set_type,word_type>  word_functor;
    localize_suffix_functor                                                                     localizer;
};

/// A functor fetching the w'th word worth of 2-bit symbols from the i-th string in a set
///
template <uint32 SYMBOL_SIZE, uint32 WORD_BITS, uint32 DOLLAR_BITS, typename string_type, typename word_type>
struct string_suffix_word_functor
{
    typedef uint32      argument_type;
    typedef word_type   result_type;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    string_suffix_word_functor(const uint64 _string_len, const string_type _string, const uint32 _w) :
        string_len(_string_len),
        string(_string),
        w(_w) {}

    /// return the w'th word of the i-th string
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    result_type operator() (const uint64 suffix_idx) const
    {
        return result_type( extract_word_generic<WORD_BITS,DOLLAR_BITS,SYMBOL_SIZE>(
            string,
            string_len,
            suffix_idx,
            w ) );
    }

    const uint64    string_len;
    string_type     string;
    uint32          w;
};

/// A functor fetching the w'th word worth of 2-bit symbols from the i-th string in a set
///
template <uint32 SYMBOL_SIZE, uint32 WORD_BITS, uint32 DOLLAR_BITS, typename storage_type, typename symbol_type, typename index_type, typename word_type>
struct string_suffix_word_functor<
    SYMBOL_SIZE, WORD_BITS, DOLLAR_BITS,
    PackedStream<storage_type,symbol_type,SYMBOL_SIZE,true,index_type>,
    word_type>
{
    typedef typename PackedStream<storage_type,symbol_type,SYMBOL_SIZE,true,index_type>::iterator   string_type;
    typedef uint2                                                                                   argument_type;
    typedef word_type                                                                               result_type;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    string_suffix_word_functor(const index_type _string_len, const string_type _string, const uint32 _w) :
        string_len(_string_len),
        string(_string),
        w(_w) {}

    /// return the w'th word of the i-th string
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    result_type operator() (const index_type suffix_idx) const
    {
        const storage_type base_words = string.stream();

        return result_type( extract_word_packed<WORD_BITS,DOLLAR_BITS,SYMBOL_SIZE>(
            base_words,
            string_len,
            string.index(),
            suffix_idx,
            w ) );
    }

    const index_type    string_len;
    string_type         string;
    uint32              w;
};

/// A binary functor calculating whether two suffixes differ (returning 1) or not (returning 0)
///
template <typename string_type>
struct string_suffix_difference
{
    typedef uint32   first_argument_type;
    typedef uint32   second_argument_type;
    typedef uint32   result_type;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    string_suffix_difference(const uint64 _string_len, const string_type _string, const uint32 _cmp_len) :
        string_len(_string_len),
        string(_string),
        cmp_len( _cmp_len ) {}

    /// return the w'th word of the i-th string
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    result_type operator() (const uint64 suffix_idx1, const uint64 suffix_idx2) const
    {
        // if one of the two suffixes is less than cmp_len, then the two suffixes must
        // necessarily differ (because no two suffixes have the same length)
        if (string_len - suffix_idx1 < cmp_len ||
            string_len - suffix_idx2 < cmp_len)
            return 1u;

        for (uint32 i = 0; i < cmp_len; ++i)
        {
            if (string[suffix_idx1 + i] != string[suffix_idx2 + i])
                return 1u;
        }
        return 0u;
    }

    const uint64    string_len;
    string_type     string;
    uint32          cmp_len;
};

/// A binary functor comparing two suffixes lexicographically
///
template <uint32 SYMBOL_SIZE, typename string_type>
struct string_suffix_less
{
    typedef uint32   first_argument_type;
    typedef uint32   second_argument_type;
    typedef uint32   result_type;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    string_suffix_less(const uint64 _string_len, const string_type _string) :
        string_len(_string_len),
        string(_string) {}

    /// return true if the first suffix is lexicographically smaller than the second, false otherwise
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    result_type operator() (const uint64 suffix_idx1, const uint64 suffix_idx2) const
    {
        const uint32 WORD_BITS   = 32u; // use 32-bit words
        const uint32 DOLLAR_BITS = 4u;  // 4 is the minimum number needed to encode up to 16 symbols per word
        const uint32 SYMBOLS_PER_WORD = symbols_per_word<SYMBOL_SIZE,WORD_BITS,DOLLAR_BITS>();

        const uint32 n_words = uint32( nvbio::min(
            (string_len - suffix_idx1),
            (string_len - suffix_idx2) ) + SYMBOLS_PER_WORD-1 ) / SYMBOLS_PER_WORD;

        // loop through all string-words
        for (uint32 w = 0; w < n_words; ++w)
        {
            string_suffix_word_functor<SYMBOL_SIZE,WORD_BITS,DOLLAR_BITS,string_type,uint32> word_functor( string_len, string, w );

            const uint32 w1 = word_functor( suffix_idx1 );
            const uint32 w2 = word_functor( suffix_idx2 );
            if (w1 < w2) return true;
            if (w1 > w2) return false;
        }
        return false;
    }

    const uint64    string_len;
    string_type     string;
};

/// given a string, return the symbol preceding each of its suffixes, or 255u to mark the
/// special $ symbol used for the first suffix.
///
template <typename string_type>
struct string_bwt_functor
{
    typedef uint64  argument_type;
    typedef uint8   result_type;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    string_bwt_functor(const uint64 _string_len, const string_type _string) :
        string_len(_string_len),
        string(_string) {}

    /// return the symbol preceding the given suffix
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    result_type operator() (const argument_type suffix_idx) const
    {
        return suffix_idx ? string[suffix_idx-1] : 255u; // use 255u to mark the dollar sign
    }

    const uint64        string_len;
    const string_type   string;
};

/// given a string set, return the symbol preceding each of its suffixes, or 255u to mark the
/// special $ symbol used for the first suffix.
///
template <typename string_set_type>
struct string_set_bwt_functor
{
    typedef uint2  argument_type;
    typedef uint8  result_type;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    string_set_bwt_functor(const string_set_type _string_set) :
        string_set(_string_set) {}

    /// return the symbol preceding the given suffix
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    result_type operator() (const argument_type local_suffix_idx) const
    {
        typedef typename string_set_type::string_type string_type;

        const uint32 string_idx = local_suffix_idx.y;
        const uint32 suffix_idx = local_suffix_idx.x;

        const string_type string = string_set[string_idx];

        return suffix_idx ? string[suffix_idx-1] : 255u; // use 255u to mark the dollar sign
    }

    /// return the last symbol of a given string
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    result_type operator() (const uint32 string_idx) const
    {
        typedef typename string_set_type::string_type string_type;

        const string_type string = string_set[string_idx];

        return string[ string.length()-1 ];
    }

    const string_set_type string_set;
};

/// A binary functor implementing some custom logic to remove singletons from a set of segment-flags
///
struct remove_singletons
{
    typedef uint32   first_argument_type;
    typedef uint32   second_argument_type;
    typedef uint32   result_type;

    /// functor operator
    ///
    /// \return         flag1 && flag2 ? 0 : 1
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    result_type operator() (const uint32 flag1, const uint32 flag2) const
    {
        return (flag1 && flag2) ? 0u : 1u;
    }
};

/// A binary functor to merge keys with new radices
///
struct merge_keys
{
    typedef uint32   first_argument_type;
    typedef uint64   second_argument_type;
    typedef uint64   result_type;

    /// functor operator
    ///
    /// \return         (key << 32u) | radix
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    result_type operator() (const first_argument_type radix, const second_argument_type key) const
    {
        return (key << 32u) | second_argument_type( radix );
    }
};

/*
template <uint32 SYMBOL_SIZE, uint32 WORD_BITS, uint32 DOLLAR_BITS, typename string_set_type>
struct dispatch_set_suffix_radices
{
    template <typename radix_iterator>
    void enact(
        const string_set_type&                  string_set,
        const SetSuffixFlattener<SYMBOL_SIZE>&  set_flattener,
        radix_iterator                          radices)
    {
        typedef typename std::iterator_traits<radix_iterator>::value_type word_type;

        thrust::transform(
            thrust::make_counting_iterator(0u),
            thrust::make_counting_iterator(0u) + set_flattener.n_suffixes,
            radices,
            global_set_suffix_word_functor<SYMBOL_SIZE,BITS,DOLLAR_BITS,string_set_type,word_type>(
                string_set,
                nvbio::device_view( set_flattener.cum_lengths ),
                nvbio::device_view( set_flattener.string_ids ),
                0u ) );
    }
};

template <uint32 SYMBOL_SIZE, uint32 WORD_BITS, uint32 DOLLAR_BITS, typename storage_type, typename index_type>
struct dispatch_set_suffix_radices<
    SYMBOL_SIZE, WORD_BITS, DOLLAR_BITS,
    ConcatenatedStringSet<PackedStream<storage_type,uint8,SYMBOL_SIZE,true,index_type>,index_type*>,
    word_type>
{
    typedef ConcatenatedStringSet<
        PackedStream<storage_type,uint8,SYMBOL_SIZE,true,index_type>,
        index_type*>        string_set_type;

    template <typename radix_iterator>
    void enact(
        const string_set_type&                  string_set,
        const SetSuffixFlattener<SYMBOL_SIZE>&  set_flattener,
        radix_iterator                          radices)
    {
        typedef typename std::iterator_traits<radix_iterator>::value_type word_type;

        thrust::transform(
            thrust::make_counting_iterator(0u),
            thrust::make_counting_iterator(0u) + set_flattener.n_suffixes,
            radices,
            global_set_suffix_word_functor<SYMBOL_SIZE,BITS,DOLLAR_BITS,string_set_type,word_type>(
                string_set,
                nvbio::device_view( set_flattener.cum_lengths ),
                nvbio::device_view( set_flattener.string_ids ),
                word_idx ) );
    }
};
*/

template <uint32 BITS, uint32 DOLLAR_BITS>
struct Bits {};

/// A helper class allowing to "flatten" the suffixes in a given string-set, i.e. to extract
/// all of them word-by-word in a flattened array.
///
template <uint32 SYMBOL_SIZE>
struct SetSuffixFlattener
{
    SetSuffixFlattener(mgpu::ContextPtr _mgpu) :
        d_scan_time(0.0f),
        d_search_time(0.0f),
        m_mgpu( _mgpu ) {}

    /// clear internal timers
    ///
    void clear_timers()
    {
        d_scan_time   = 0.0f;
        d_search_time = 0.0f;
    }

    /// reserve storage
    ///
    void reserve(const uint32 n_strings, const uint32 n_suffixes)
    {
        alloc_storage( cum_lengths, n_strings );
        alloc_storage( string_ids,  n_suffixes );
    }

    /// return the amount of device memory needed
    ///
    uint64 needed_device_memory(const uint32 n_strings, const uint32 n_suffixes) const
    {
        return (n_strings + n_suffixes) * sizeof(uint32);
    }

    /// initialize this flattener, building the auxiliary data-structures needed
    /// to extract the radices
    ///
    template <typename string_set_type>
    void set(const string_set_type& string_set, const bool empty_suffixes = true)
    {
        const uint32 n = string_set.size();

        cuda::Timer timer;
        timer.start();

        // compute the cumulative sum of the string lengths in the set - we will use this for
        // building the map: (global suffix index -> string index)
        alloc_storage( cum_lengths, n );

        cuda::inclusive_scan(
            n,
            thrust::make_transform_iterator( thrust::make_counting_iterator(0u), length_functor<string_set_type>( string_set, empty_suffixes ) ),
            cum_lengths.begin(),
            thrust::plus<uint32>(),
            temp_storage );

        // compute the number of suffixes
        n_suffixes = cum_lengths[n-1];

        timer.stop();
        d_scan_time += timer.seconds();

        timer.start();

        // assign the string id to each suffix - this is done by a simple binary search on the suffix index
        // in the vector of cumulative string lengths
        alloc_storage( string_ids, n_suffixes );

        // find the end of each bin of values
        mgpu::SortedSearch<mgpu::MgpuBoundsLower>(
            thrust::make_counting_iterator<uint32>(0u),
            n_suffixes,
            thrust::make_transform_iterator( cum_lengths.begin(), minus_one() ),
            n,
            string_ids.begin(),
            *m_mgpu );

        timer.stop();
        d_search_time += timer.seconds();
    }

    /// extract the given radix of all suffixes
    ///
    template <uint32 BITS, uint32 DOLLAR_BITS, typename string_set_type, typename index_iterator, typename radix_iterator>
    void flatten(
        const string_set_type&          string_set,
        const uint32                    word_idx,
        const Bits<BITS,DOLLAR_BITS>    word_bits,
        const index_iterator            indices,
              radix_iterator            radices)
    {
        typedef typename std::iterator_traits<radix_iterator>::value_type word_type;

        thrust::transform(
            indices,
            indices + n_suffixes,
            radices,
            global_set_suffix_word_functor<SYMBOL_SIZE,BITS,DOLLAR_BITS,string_set_type,word_type>(
                string_set,
                nvbio::device_view( cum_lengths ),
                nvbio::device_view( string_ids ),
                word_idx ) );
    }

    /// compute the maximum suffix length among the specified range
    ///
    template <typename string_set_type>
    uint32 max_length(
        const string_set_type&  string_set, const bool empty_suffixes = true)
    {
        // compute the maximum string length in the set
        return cuda::reduce(
            uint32( string_set.size() ),
            thrust::make_transform_iterator(
                thrust::make_counting_iterator<uint32>(0u),
                length_functor<string_set_type>( string_set, empty_suffixes ) ),
            thrust::maximum<uint32>(),
            temp_storage );
    }

    /// compute the maximum suffix length among the specified range
    ///
    template <typename string_set_type, typename index_iterator>
    uint32 max_length(
        const string_set_type&  string_set,
        const index_iterator    indices_begin,
        const index_iterator    indices_end)
    {
        // TODO: this function is conservative, in the sense it returns the maximum *string* length;
        // however, each suffix might be shorter than the string it belongs to.
        return indices_end <= indices_begin ? 0u :
            cuda::reduce(
                indices_end - indices_begin,
                thrust::make_transform_iterator(
                    thrust::make_permutation_iterator( string_ids.begin(), indices_begin ),
                    length_functor<string_set_type>( string_set, false ) ),
                thrust::maximum<uint32>(),
                temp_storage );
    }

    /// return the amount of used device memory
    ///
    uint64 allocated_device_memory() const
    {
        return
            cum_lengths.size() * sizeof(uint32) +
            string_ids.size() * sizeof(uint32) +
            temp_storage.size() * sizeof(uint8);
    }

    uint32                          n_suffixes;         ///< number of suffixes in the string set
    thrust::device_vector<uint32>   cum_lengths;        ///< cumulative string lengths
    thrust::device_vector<uint32>   string_ids;         ///< a vector containing the string index corresponding
                                                        ///  to each flattened suffixes; i.e. if the set contains
                                                        ///  3 strings of length (3, 2, 5), the string ids will be
                                                        ///  the vector (0,0,0,1,1,2,2,2,2,2).
    thrust::device_vector<uint8>    temp_storage;

    float                           d_scan_time;
    float                           d_search_time;
    mgpu::ContextPtr                m_mgpu;
};

/// A helper class to load a chunk of a string_set from the host onto the device
///
template <uint32 SYMBOL_SIZE, bool BIG_ENDIAN, typename storage_type, typename offsets_iterator, typename input_tag, typename output_tag>
struct ChunkLoader {};

/// A helper class to load a chunk of a string_set from the host onto the device
///
template <uint32 SYMBOL_SIZE, bool BIG_ENDIAN, typename storage_type, typename offsets_iterator>
struct ChunkLoader<SYMBOL_SIZE,BIG_ENDIAN,storage_type,offsets_iterator,host_tag,device_tag>
{
    // infer the word type
    typedef typename std::iterator_traits<storage_type>::value_type     word_type;
    typedef typename std::iterator_traits<offsets_iterator>::value_type index_type;

    typedef cuda::load_pointer<word_type,cuda::LOAD_LDG> word_pointer;

    typedef PackedStream<word_pointer,uint8,SYMBOL_SIZE,BIG_ENDIAN> packed_stream_type;
    typedef typename packed_stream_type::iterator                   packed_stream_iterator;

    typedef ConcatenatedStringSet<packed_stream_iterator,uint32*>   chunk_set_type;

    // infer the word size
    static const uint32 SYMBOLS_PER_WORD = uint32(8u*sizeof(word_type))/SYMBOL_SIZE;

    uint64 needed_device_memory(const uint32 max_strings, const uint32 max_symbols) const
    {
        const uint32 max_words = util::divide_ri( max_symbols, SYMBOLS_PER_WORD ) + 2;

        return (max_strings+1) * sizeof(uint32) +
                max_words      * sizeof(word_type);
    }

    void reserve(const uint32 max_strings, const uint32 max_symbols)
    {
        const uint32 max_words = util::divide_ri( max_symbols, SYMBOLS_PER_WORD ) + 2;

        alloc_storage( h_chunk_offsets, max_strings+1 );
        alloc_storage( d_chunk_offsets, max_strings+1 );
        alloc_storage( d_chunk_string,  max_words );
    }

    chunk_set_type load(
        const ConcatenatedStringSet<
            typename PackedStream<storage_type,uint8,SYMBOL_SIZE,BIG_ENDIAN,index_type>::iterator,
            offsets_iterator>       string_set,
        const uint32                chunk_begin,
        const uint32                chunk_end)
    {
        const uint32 chunk_size = chunk_end - chunk_begin;

        alloc_storage( h_chunk_offsets, chunk_size+1 );
        alloc_storage( d_chunk_offsets, chunk_size+1 );

        // find the words overlapped by the chunk
        const uint64 begin_index = string_set.offsets()[ chunk_begin ];
        const uint64 end_index   = string_set.offsets()[ chunk_end ];
        const uint64 begin_word  = (begin_index / SYMBOLS_PER_WORD);
        const uint64 end_word    = (end_index + SYMBOLS_PER_WORD-1) / SYMBOLS_PER_WORD;
        const uint32 chunk_words = uint32( end_word - begin_word );

        const word_type* base_words = string_set.base_string().stream();

        alloc_storage( d_chunk_string, chunk_words );

        // copy them to the device
        thrust::copy(
            base_words + begin_word,
            base_words + begin_word + chunk_words,
            d_chunk_string.begin() );

        // build the host offsets
        uint32 chunk_symbols = uint32( begin_index % SYMBOLS_PER_WORD );
        h_chunk_offsets[0] = chunk_symbols;
        for (uint32 i = 0; i < chunk_size; ++i)
        {
            chunk_symbols += string_set[ chunk_begin + i ].size();
            h_chunk_offsets[i+1] = chunk_symbols;
        }

        // copy the offsets to the device
        thrust::copy(
            h_chunk_offsets.begin(),
            h_chunk_offsets.begin() + chunk_size+1,
            d_chunk_offsets.begin() );

        // finally assemble the device chunk string-set
        packed_stream_type d_packed_stream( word_pointer( nvbio::plain_view( d_chunk_string ) ) );
        return chunk_set_type(
            chunk_size,
            d_packed_stream.begin(),
            nvbio::plain_view( d_chunk_offsets ) );
    }

    thrust::host_vector<uint32>      h_chunk_offsets;
    thrust::device_vector<word_type> d_chunk_string;
    thrust::device_vector<uint32>    d_chunk_offsets;
};

/// A helper class to load a chunk of a string_set from the host onto the device
///
template <uint32 SYMBOL_SIZE, bool BIG_ENDIAN, typename storage_type, typename offsets_iterator, typename system_tag>
struct ChunkLoader<SYMBOL_SIZE,BIG_ENDIAN,storage_type,offsets_iterator,system_tag,system_tag>
{
    typedef typename std::iterator_traits<offsets_iterator>::value_type index_type;

    typedef const ConcatenatedStringSet<
        typename PackedStream<storage_type,uint8,SYMBOL_SIZE,BIG_ENDIAN,index_type>::iterator,
        offsets_iterator>           string_set_type;

    typedef string_set_type         chunk_set_type;

    chunk_set_type load(
        const string_set_type       string_set,
        const uint32                chunk_begin,
        const uint32                chunk_end)
    {
        // assemble the device chunk string-set
        return chunk_set_type(
            uint32( chunk_end - chunk_begin ),
            string_set.base_string(),
            string_set.offsets() + chunk_begin );
    }
};

/// extract the given radix from the given suffixes of a string
///
template <uint32 SYMBOL_SIZE, uint32 BITS, uint32 DOLLAR_BITS, typename string_type, typename index_iterator, typename radix_iterator>
void flatten_string_suffixes(
    const uint64            string_len,
    const string_type&      string,
    const uint32            word_idx,
    const index_iterator    indices_begin,
    const index_iterator    indices_end,
          radix_iterator    radices)
{
    typedef typename std::iterator_traits<radix_iterator>::value_type word_type;

    thrust::transform(
        indices_begin,
        indices_end,
        radices,
        string_suffix_word_functor<SYMBOL_SIZE, BITS, DOLLAR_BITS,string_type,word_type>(
            string_len,
            string,
            word_idx ) );
}

/// A context class to perform suffix bucketing
///
template <uint32 SYMBOL_SIZE, uint32 N_BITS, uint32 DOLLAR_BITS>
struct StringSuffixBucketer
{
    typedef uint32 word_type;

    StringSuffixBucketer() : d_setup_time(0.0f), d_flatten_time(0.0f), d_count_sort_time(0.0f), d_collect_sort_time(0.0f), d_remap_time(0.0f), d_copy_time(0.0f), d_filter_time(0.0f) {}

    /// count the number of suffixes falling in each bucket, where the buckets
    /// are defined by the first n_bits of the suffix
    ///
    template <typename suffix_iterator, typename string_type>
    void count(
        const uint32            n_suffixes,
        const suffix_iterator   suffixes,
        const uint32            string_length,
        const string_type&      string)
    {
        cuda::Timer timer;

        const uint32 n_buckets = 1u << (N_BITS);

        // initialize the temporary and output vectors
        alloc_storage( d_indices, n_suffixes * 2u );
        alloc_storage( d_radices, n_suffixes * 2u );
        alloc_storage( d_buckets, n_buckets );

        timer.start();

        // extract the first radix word from each of the suffixes
        flatten_string_suffixes<SYMBOL_SIZE, N_BITS,DOLLAR_BITS>(
            string_length,
            string,
            0u,                     // load the first word
            suffixes,
            suffixes + n_suffixes,
            d_radices.begin() );

        timer.stop();
        d_flatten_time += timer.seconds();

        timer.start();

        // sort the radices so as to make binning easy
        cuda::SortBuffers<word_type*>   sort_buffers;
        cuda::SortEnactor               sort_enactor;

        sort_buffers.selector = 0;
        sort_buffers.keys[0]  = nvbio::device_view( d_radices );
        sort_buffers.keys[1]  = nvbio::device_view( d_radices ) + n_suffixes;
        sort_enactor.sort( n_suffixes, sort_buffers, 0u, N_BITS );
        //thrust::sort( d_radices.begin(), d_radices.begin() + n_suffixes );

        timer.stop();
        d_count_sort_time += timer.seconds();

        // initialize the bucket counters
        thrust::fill( d_buckets.begin(), d_buckets.end(), 0u );

        // compute the number of effectively used buckets looking at the last non-empty one
        const uint32 n_used_buckets = d_radices[ sort_buffers.selector * n_suffixes + n_suffixes-1 ] + 1u;

        // find the end of each bin of values
        thrust::upper_bound(
            d_radices.begin() + sort_buffers.selector * n_suffixes,
            d_radices.begin() + sort_buffers.selector * n_suffixes + n_suffixes,
            thrust::make_counting_iterator<uint32>(0u),
            thrust::make_counting_iterator<uint32>(0u) + n_used_buckets,
            d_buckets.begin() );

        // compute the histogram by taking differences of the cumulative histogram
        thrust::adjacent_difference(
            d_buckets.begin(), d_buckets.begin() + n_used_buckets,
            d_buckets.begin());
    }

    /// collect the suffixes falling in a given set of buckets, where the buckets
    /// are defined by the first n_bits of the suffix
    ///
    template <typename suffix_iterator, typename string_type, typename bucketmap_iterator, typename output_iterator>
    uint32 collect(
        const uint32                n_suffixes,
        const suffix_iterator       suffixes,
        const uint64                string_length,
        const string_type&          string,
        const uint32                bucket_begin,
        const uint32                bucket_end,
        const bucketmap_iterator    bucketmap,
              output_iterator       output_radices,
              output_iterator       output_indices)
    {
        cuda::Timer timer;

        const uint32 n_buckets  = 1u << N_BITS;

        // initialize the temporary and output vectors
        alloc_storage( d_indices, n_suffixes * 2u );
        alloc_storage( d_radices, n_suffixes * 2u );
        alloc_storage( d_buckets, n_buckets );

        timer.start();

        // extract the first radix word from each of the suffixes
        flatten_string_suffixes<SYMBOL_SIZE,N_BITS,DOLLAR_BITS>(
            string_length,
            string,
            0u,                     // load the first word
            suffixes,
            suffixes + n_suffixes,
            d_radices.begin() );

        timer.stop();
        d_flatten_time += timer.seconds();

        timer.start();

        // determine if a radix is in the given bucket range
        const priv::in_range_functor in_range = priv::in_range_functor( bucket_begin, bucket_end );

        // retain only suffixes whose radix is between the specified buckets
        const uint32 n_collected = cuda::copy_flagged(
            n_suffixes,
            thrust::make_zip_iterator( thrust::make_tuple( suffixes, d_radices.begin() ) ),
            thrust::make_transform_iterator( d_radices.begin(), in_range ),
            thrust::make_zip_iterator( thrust::make_tuple( d_indices.begin(), d_radices.begin() ) ) + n_suffixes,
            d_temp_storage );

        timer.stop();
        d_filter_time += timer.seconds();

        timer.start();

        // remap the collected radices
        thrust::gather(
            d_radices.begin() + n_suffixes,
            d_radices.begin() + n_suffixes + n_collected,
            bucketmap,
            d_radices.begin() + n_suffixes );

        timer.stop();
        d_remap_time += timer.seconds();

        timer.start();

        // sort the radices so as to make binning easy
        cuda::SortBuffers<word_type*,uint64*>   sort_buffers;
        cuda::SortEnactor                       sort_enactor;

        sort_buffers.selector  = 0;
      //#define SORT_BY_BUCKETS
      #if defined(SORT_BY_BUCKETS)
        sort_buffers.keys[0]   = nvbio::device_view( d_radices ) + n_suffixes;
        sort_buffers.keys[1]   = nvbio::device_view( d_radices );
        sort_buffers.values[0] = (uint64*)nvbio::device_view( d_indices ) + buffer_stride;
        sort_buffers.values[1] = (uint64*)nvbio::device_view( d_indices );
        sort_enactor.sort( n_collected, sort_buffers, 0u, N_BITS );
      #endif

        timer.stop();
        d_collect_sort_time += timer.seconds();

        //
        // copy all the indices inside the range to the output
        //

        //alloc_storage( output_suffixes, n_suffixes );
        //alloc_storage( output_radices,  n_suffixes );

        timer.start();

        // the buffer selector had inverted semantics
        sort_buffers.selector = 1 - sort_buffers.selector;

        // and copy everything to the output
        thrust::copy(
            d_indices.begin() + sort_buffers.selector * n_suffixes,
            d_indices.begin() + sort_buffers.selector * n_suffixes + n_collected,
            output_indices );

        // and copy everything to the output
        thrust::copy(
            d_radices.begin() + sort_buffers.selector * n_suffixes,
            d_radices.begin() + sort_buffers.selector * n_suffixes + n_collected,
            output_radices );

        timer.stop();
        d_copy_time += timer.seconds();

        return n_collected;
    }

    thrust::device_vector<uint32>       d_indices;
    thrust::device_vector<word_type>    d_radices;
    thrust::device_vector<uint32>       d_buckets;
    thrust::device_vector<uint8>        d_temp_storage;
    float                               d_setup_time;
    float                               d_flatten_time;
    float                               d_count_sort_time;
    float                               d_collect_sort_time;
    float                               d_remap_time;
    float                               d_copy_time;
    float                               d_filter_time;
};

//
// A host-side radix extractor context
//
template <typename string_set_type, uint32 SYMBOL_SIZE, uint32 DOLLAR_BITS, uint32 WORD_BITS>
struct HostStringSetRadices
{
    HostStringSetRadices(const string_set_type string_set) : m_string_set( string_set ) {}

    /// return the number of words needed to represent a given string length
    ///
    uint32 num_words(const uint32 max_string_len) const
    {
        const uint32 SYMBOLS_PER_WORD = priv::symbols_per_word<SYMBOL_SIZE,WORD_BITS,DOLLAR_BITS>();

        return (max_string_len + SYMBOLS_PER_WORD-1) / SYMBOLS_PER_WORD;
    }

    /// needed amount of device storage
    ///
    uint64 needed_device_memory(const uint32 n_suffixes) const
    {
        return n_suffixes * sizeof(uint8) + // d_symbols
               n_suffixes * sizeof(uint32); // d_active_suffixes
    }

    /// reserve any temporary space for the given amount of suffixes
    ///
    void reserve(const uint32 n_suffixes, const uint32 block_size)
    {
        try
        {
            d_active_suffixes.resize( n_suffixes );
            d_symbols.resize( n_suffixes );
            h_symbols.resize( n_suffixes );
            h_active_suffixes.resize( n_suffixes );
            m_block.resize( n_suffixes * block_size );
        }
        catch (...)
        {
            log_error(stderr, "HostStringSetRadices::reserve() : allocation failed!\n");
            throw;
        }
    }

    /// initialize the suffixes to extract
    ///
    void init(const uint32 n_suffixes, const uint2* _h_suffixes, const uint2* _d_suffixes)
    {
        m_suffixes = _h_suffixes;
        d_suffixes = thrust::device_ptr<const uint2>( _d_suffixes );
    }

    /// initialize extraction of a slice of words
    ///
    void init_slice(
        const uint32  n_indices,
        const uint32* d_indices,
        const uint32  word_block_begin,
        const uint32  word_block_end)
    {
        try
        {
            if (d_indices == NULL)
            {
                // extract the given radix word from each of the partially sorted suffixes on the host
                priv::extract_radices(
                    m_string_set,
                    n_indices,
                    word_block_begin,
                    word_block_end,
                    WORD_BITS,
                    m_suffixes,
                    &m_block[0],
                    word_block_begin == 0 ? &h_symbols[0] : NULL ); // on the first iteration, load the BWT symbols too
            }
            else
            {
                // gather the list of active suffixes
                thrust::gather(
                    thrust::device_ptr<const uint32>( d_indices ),
                    thrust::device_ptr<const uint32>( d_indices ) + n_indices,
                    d_suffixes,
                    d_active_suffixes.begin() );

                // copy the list of active suffixes to the host
                thrust::copy(
                    d_active_suffixes.begin(),
                    d_active_suffixes.begin() + n_indices,
                    h_active_suffixes.begin() );

                // extract the given radix word from each of the partially sorted suffixes on the host
                priv::extract_radices(
                    m_string_set,
                    n_indices,
                    word_block_begin,
                    word_block_end,
                    WORD_BITS,
                    &h_active_suffixes[0],
                    &m_block[0] );
            }
        }
        catch (...)
        {
            log_error(stderr, "HostStringSetRadices::init_slice() : exception caught!\n");
            throw;
        }
    }

    /// extract the radices corresponding to a given word of the given suffixes
    ///
    /// \param n_indices        the input number of suffixes
    /// \param d_indices        a device vector of the indices to extract
    /// \param word_idx         the word index to extract
    /// \param word_begin       the beginning of the current slice range
    /// \param word_idx         the end of the current slice range
    /// \param d_radices        the destination device array to hold the output
    ///
    void extract(
        const uint32  n_indices,
        const uint32* d_indices,
        const uint32  word_idx,
        const uint32  word_block_begin,
        const uint32  word_block_end,
        uint32*       d_radices) const
    {
        try
        {
            // and copy them to the device
            thrust::copy(
                m_block.begin() + n_indices * (word_idx - word_block_begin),
                m_block.begin() + n_indices * (word_idx - word_block_begin) + n_indices,
                thrust::device_ptr<uint32>( d_radices ) );
        }
        catch (...)
        {
            log_error(stderr, "HostStringSetRadices::extract() : exception caught!\n");
            throw;
        }
    }

    /// extract the bwt of the given block
    ///
    void dollar_bwt(
        const uint32 begin,
        const uint32 end,
        uint8* h_bwt)
    {
        const int n_strings = int( end - begin );

        // fetch the BWT symbols for the given strings
        #pragma omp parallel for
        for (int i = 0; i < n_strings; ++i)
        {
            const priv::string_set_bwt_functor<string_set_type> bwt( m_string_set );
            h_bwt[i] = bwt( i + begin );
        }
    }

    /// extract the bwt of the given block
    ///
    void bwt(
        const uint32  n_suffixes,
        const uint32* d_indices,
              uint8*  h_bwt,
              uint8*  d_bwt)
    {
        try
        {
            if (d_indices != NULL)
            {
              #if 0
                // fetch the BWT symbols for this block of suffixes
                #pragma omp parallel for
                for (int i = 0; i < n_suffixes; ++i)
                {
                    const priv::string_set_bwt_functor<string_set_type> bwt( m_string_set );
                    h_symbols[i] = bwt( m_suffixes[i] );
                }
              #endif

              #if 0
                alloc_storage( m_block, n_suffixes );

                // re-purpose the radix-block storage
                uint32* h_indices = &m_block[0];

                // copy the sorted indices to the host
                thrust::copy(
                    thrust::device_ptr<const uint32>( d_indices ),
                    thrust::device_ptr<const uint32>( d_indices ) + n_suffixes,
                    h_indices );

                // and compute the bwt of the block by gathering the symbols in suffix-sorted order
                thrust::gather(
                    h_indices,
                    h_indices + n_suffixes,
                    h_symbols.begin(),
                    h_bwt );
              #else
                alloc_storage( d_symbols, n_suffixes );

                // copy the symbols to the device
                thrust::copy(
                    h_symbols.begin(),
                    h_symbols.begin() + n_suffixes,
                    d_symbols.begin() );

                // gather the symbols in proper order
                thrust::gather(
                    thrust::device_ptr<const uint32>( d_indices ),
                    thrust::device_ptr<const uint32>( d_indices ) + n_suffixes,
                    d_symbols.begin(),
                    thrust::device_ptr<uint8>( d_bwt ) );

                // and copy the sorted symbols back to the host
                thrust::copy(
                    thrust::device_ptr<uint8>( d_bwt ),
                    thrust::device_ptr<uint8>( d_bwt ) + n_suffixes,
                    h_bwt );
                #endif
            }
            else
            {
                // fetch the BWT symbols for this block of suffixes
                #pragma omp parallel for
                for (int i = 0; i < n_suffixes; ++i)
                {
                    const priv::string_set_bwt_functor<string_set_type> bwt( m_string_set );
                    h_bwt[i] = bwt( m_suffixes[i] );
                }

                // and copy the sorted symbols back to the device
                thrust::copy(
                    h_bwt,
                    h_bwt + n_suffixes,
                    thrust::device_ptr<uint8>( d_bwt ) );
            }
        }
        catch (...)
        {
            log_error(stderr, "HostStringSetRadices::bwt() : exception caught!\n");
            throw;
        }
    }

    /// return the amount of used device memory
    ///
    uint64 allocated_device_memory() const
    {
        return
            d_active_suffixes.size() * sizeof(uint2) +
            d_symbols.size()         * sizeof(uint8);
    }

    /// return the amount of used device memory
    ///
    uint64 allocated_host_memory() const
    {
        return
            m_block.size()           * sizeof(uint2) +
            h_active_suffixes.size() * sizeof(uint2) +
            h_symbols.size()         * sizeof(uint8);
    }

    string_set_type                 m_string_set;
    const uint2*                    m_suffixes;
    thrust::device_ptr<const uint2> d_suffixes;
    thrust::device_vector<uint2>    d_active_suffixes;
    thrust::device_vector<uint8>    d_symbols;
    thrust::host_vector<uint2>      h_active_suffixes;
    thrust::host_vector<uint8>      h_symbols;
    thrust::host_vector<uint32>     m_block;
};

//
// A host-side radix extractor context
//
template <typename string_set_type, uint32 SYMBOL_SIZE, uint32 DOLLAR_BITS, uint32 WORD_BITS>
struct DeviceStringSetRadices
{
    DeviceStringSetRadices() {}
    DeviceStringSetRadices(const string_set_type string_set) : m_string_set( string_set ) {}

    /// return the number of words needed to represent a given string length
    ///
    uint32 num_words(const uint32 max_string_len) const
    {
        const uint32 SYMBOLS_PER_WORD = priv::symbols_per_word<SYMBOL_SIZE,WORD_BITS,DOLLAR_BITS>();

        return (max_string_len + SYMBOLS_PER_WORD-1) / SYMBOLS_PER_WORD;
    }

    /// needed amount of device storage
    ///
    uint64 needed_device_memory(const uint32 n_suffixes) const
    {
        return n_suffixes; // d_symbols
    }

    /// reserve any temporary space for the given amount of suffixes
    ///
    void reserve(const uint32 n_suffixes, const uint32 slice_size)
    {
        d_symbols.resize( n_suffixes );
    }

    /// set string set
    ///
    void set(const string_set_type string_set) { m_string_set = string_set; }

    /// initialize the suffixes to extract
    ///
    void init(const uint32 n_suffixes, const uint2* _h_suffixes, const uint2* _d_suffixes)
    {
        d_suffixes = thrust::device_ptr<const uint2>( _d_suffixes );
    }

    /// initialize extraction of a slice of words
    ///
    void init_slice(
        const uint32  n_indices,
        const uint32* d_indices,
        const uint32  word_block_begin,
        const uint32  word_block_end) {}
        
    /// extract the radices corresponding to a given word of the given suffixes
    ///
    /// \param n_indices        the input number of suffixes
    /// \param d_indices        a device vector of the indices to extract
    /// \param word_idx         the word index to extract
    /// \param word_begin       the beginning of the current slice range
    /// \param word_idx         the end of the current slice range
    /// \param d_radices        the destination device array to hold the output
    ///
    void extract(
        const uint32  n_indices,
        const uint32* d_indices,
        const uint32  word_idx,
        const uint32  word_block_begin,
        const uint32  word_block_end,
        uint32*       d_radices) const
    {
        // extract the given radix word from each of the partially sorted suffixes in a device temp buffer
        priv::local_set_suffix_word_functor<SYMBOL_SIZE,WORD_BITS,DOLLAR_BITS,string_set_type,uint32> word_functor( m_string_set, word_idx );

        if (d_indices == NULL)
        {
            thrust::copy(
                thrust::make_transform_iterator( d_suffixes, word_functor ),
                thrust::make_transform_iterator( d_suffixes, word_functor ) + n_indices,
                thrust::device_ptr<uint32>( d_radices ) );
        }
        else
        {
            thrust::copy(
                thrust::make_transform_iterator( thrust::make_permutation_iterator( d_suffixes, thrust::device_ptr<const uint32>( d_indices ) ), word_functor ),
                thrust::make_transform_iterator( thrust::make_permutation_iterator( d_suffixes, thrust::device_ptr<const uint32>( d_indices ) ), word_functor ) + n_indices,
                thrust::device_ptr<uint32>( d_radices ) );
        }
    }

    /// extract the bwt of the given block
    ///
    void dollar_bwt(
        const uint32 begin,
        const uint32 end,
        uint8*       h_bwt)
    {
        const int n_strings = end - begin;

        alloc_storage( d_symbols, n_strings );

        // fetch the BWT symbols for the given strings
        thrust::transform(
            thrust::make_counting_iterator<uint32>(begin),
            thrust::make_counting_iterator<uint32>(end),
            d_symbols.begin(),
            priv::string_set_bwt_functor<string_set_type>( m_string_set ) );

        // and copy the result to the host
        thrust::copy(
            d_symbols.begin(),
            d_symbols.begin() + n_strings,
            h_bwt );
    }

    /// extract the bwt of the given block
    ///
    void bwt(
        const uint32  n_suffixes,
        const uint32* d_indices,
              uint8*  h_bwt,
              uint8*  d_bwt)
    {
        if (d_indices != NULL)
        {
            alloc_storage( d_symbols, n_suffixes );

            // fetch the BWT symbols for this block of suffixes
            thrust::transform(
                d_suffixes,
                d_suffixes + n_suffixes,
                d_symbols.begin(),
                priv::string_set_bwt_functor<string_set_type>( m_string_set ) );

            // and compute the bwt of the block by gathering the symbols in suffix-sorted order
            thrust::gather(
                thrust::device_ptr<const uint32>( d_indices ),
                thrust::device_ptr<const uint32>( d_indices ) + n_suffixes,
                d_symbols.begin(),
                thrust::device_ptr<uint8>( d_bwt ) );
        }
        else
        {
            // fetch the BWT symbols for this block of suffixes
            thrust::transform(
                d_suffixes,
                d_suffixes + n_suffixes,
                thrust::device_ptr<uint8>( d_bwt ),
                priv::string_set_bwt_functor<string_set_type>( m_string_set ) );
        }

        // and copy the result to the host
        thrust::copy(
            thrust::device_ptr<uint8>( d_bwt ),
            thrust::device_ptr<uint8>( d_bwt ) + n_suffixes,
            h_bwt );
    }

    /// return the amount of used device memory
    ///
    uint64 allocated_device_memory() const
    {
        return d_symbols.size() * sizeof(uint8);
    }

    /// return the amount of used device memory
    ///
    uint64 allocated_host_memory() const
    {
        return 0u;
    }

    string_set_type                 m_string_set;
    thrust::device_ptr<const uint2> d_suffixes;
    thrust::device_vector<uint8>    d_symbols;
};

/// Collect dollar symbols out of a BWT + SA block
///
struct DollarExtractor
{
    /// constructor
    ///
    DollarExtractor() :
        offset(0),
        n_dollars(0) {}

    /// process a batch of BWT symbols
    ///
    uint32 extract(
        const uint32  n_suffixes,
        const uint8*  h_bwt,
        const uint8*  d_bwt,
        const uint2*  h_suffixes,
        const uint2*  d_suffixes,
        const uint32* d_indices);

    uint64                          offset;
    uint32                          n_dollars;

    thrust::device_vector<uint64>   d_dollar_ranks;
    thrust::device_vector<uint32>   d_dollar_indices;
    thrust::device_vector<uint64>   d_dollars;
    thrust::host_vector<uint64>     h_dollar_ranks;
    thrust::host_vector<uint64>     h_dollars;
    thrust::device_vector<uint8>    d_temp_storage;
};

// ------------------------------------------------------------------------------------------------------------- //
// the following functions implement device_copy() and device_scatter() - special-purpose functions to copy
// and scatter a set of symbols to a packed stream.
// ------------------------------------------------------------------------------------------------------------- //

/// a simple auxiliary kernel to perform generic device-to-device copies, specialized for packed streams
///
template <typename input_iterator, typename output_iterator, typename index_type>
__global__ void simple_device_copy_kernel(
    const uint32            n,
    const input_iterator    input,
          output_iterator   output,
    const index_type        offset)
{
    const uint32 thread_id = threadIdx.x + blockIdx.x*blockDim.x;

    if (thread_id < n)
        output[offset + thread_id] = input[thread_id];
}

/// a simple auxiliary kernel to perform generic device-to-device copies, specialized for packed streams
///
template <typename input_iterator, typename storage_type, uint32 SYMBOL_SIZE, bool BIG_ENDIAN, typename index_type>
__global__ void packed_device_copy_kernel(
    const uint32                                                                                        n,
    const input_iterator                                                                                input,
          PackedStream<storage_type,uint8,SYMBOL_SIZE,BIG_ENDIAN,index_type>                            output,
    const index_type                                                                                    offset)
{
    const uint32 thread_id = threadIdx.x + blockIdx.x*blockDim.x;

    //
    // care must be used to avoid write-conflicts, hence we assign all symbols belonging
    // to the same output word to a single thread
    //
    typedef typename PackedStream<storage_type,uint8,SYMBOL_SIZE,BIG_ENDIAN,index_type>::storage_type word_type;
    const uint32 SYMBOLS_PER_WORD = (8u*sizeof(word_type))/SYMBOL_SIZE;

    const uint32 word_offset = uint32( (offset + output.index()) & (SYMBOLS_PER_WORD-1) );
    const uint32 elem_begin = thread_id ? (thread_id+0) * SYMBOLS_PER_WORD - word_offset : 0u;
    const uint32 elem_end   = nvbio::min( (thread_id+1) * SYMBOLS_PER_WORD - word_offset, n );

    if (elem_begin < n)
    {
        for (uint32 i = elem_begin; i < elem_end; ++i)
            output[offset+i] = input[i];
    }
}

/// a dispatcher for device_copy
///
template <typename input_iterator, typename output_iterator, typename index_type>
struct device_copy_dispatch
{
    /// copy n elements from the input stream to the output
    ///
    static void copy(
            const uint32            n,
        const input_iterator        input,
        const output_iterator       output,
        const index_type            offset)
    {
        const uint32 batch_size = cuda::max_grid_size();
        for (uint32 batch_begin = 0; batch_begin < n; batch_begin += batch_size)
        {
            const uint32 batch_end = nvbio::min( batch_begin + batch_size, n );

            const uint32 blockdim = 128;
            const uint32 n_blocks = util::divide_ri( batch_end - batch_begin, blockdim );
            simple_device_copy_kernel<<<n_blocks,blockdim>>>( n, input, output, offset );
        }
    }
};

/// a dispatcher for device_copy
///
template <typename input_iterator, typename storage_type, uint32 SYMBOL_SIZE, bool BIG_ENDIAN, typename index_type>
struct device_copy_dispatch<
    input_iterator,
    PackedStream<storage_type,uint8,SYMBOL_SIZE,BIG_ENDIAN,index_type>,
    index_type>
{
    typedef PackedStream<storage_type,uint8,SYMBOL_SIZE,BIG_ENDIAN,index_type>  output_iterator;

    /// copy n elements from the input stream to the output
    ///
    static void copy(
            const uint32            n,
        const input_iterator        input,
        const output_iterator       output,
        const index_type            offset)
    {
        typedef typename PackedStream<storage_type,uint8,SYMBOL_SIZE,BIG_ENDIAN,index_type>::storage_type word_type;
        const uint32 SYMBOLS_PER_WORD = (8u*sizeof(word_type))/SYMBOL_SIZE;

        const uint32 batch_size = cuda::max_grid_size();
        for (uint32 batch_begin = 0; batch_begin < n; batch_begin += batch_size)
        {
            const uint32 batch_end = nvbio::min( batch_begin + batch_size, n );

            const uint32 blockdim = 128;
            const uint32 n_words  = util::divide_ri( batch_end - batch_begin, SYMBOLS_PER_WORD ) + 1u;
            const uint32 n_blocks = util::divide_ri( n_words, blockdim );

            packed_device_copy_kernel<<<n_blocks,blockdim>>>( batch_end - batch_begin, input, output, offset + batch_begin );
        }
    }
};

/// copy a set of n symbols from a given input stream to a given output stream
///
template <typename input_iterator, typename output_iterator, typename index_type>
void device_copy(
        const uint32            n,
    const input_iterator        input,
    const output_iterator       output,
    const index_type            offset)
{
    device_copy_dispatch<input_iterator,output_iterator,index_type>::copy( n, input, output, offset );
}

/// an auxiliary kernel to scatter a set of symbols into a sparse set of slots of a given output stream;
/// this kernel copies a full range of symbols per thread, where individual ranges are guaranteed to
/// touch distinct words of the underlying storage where the output is packed.
///
template <typename input_iterator, typename slot_iterator, typename range_iterator, typename storage_type, uint32 SYMBOL_SIZE, bool BIG_ENDIAN, typename index_type>
__global__ void device_scatter_kernel(
    const uint32                                                                                        begin,
    const uint32                                                                                        end,
    const range_iterator                                                                                ranges,
    const input_iterator                                                                                input,
    const slot_iterator                                                                                 slots,
          PackedStream<storage_type,uint8,SYMBOL_SIZE,BIG_ENDIAN,index_type>                            output)
{
    const uint32 thread_id = threadIdx.x + blockIdx.x*blockDim.x;
    const uint32 idx = thread_id + begin;

    if (idx >= end)
        return;

    //
    // care must be used to avoid write-conflicts, hence we assign all symbols belonging
    // to the same output word to a single thread
    //
    const uint32 elem_begin = idx ? ranges[ idx-1 ] : 0u;
    const uint32 elem_end   =       ranges[ idx   ];

    for (uint32 i = elem_begin; i < elem_end; ++i)
    {
        const uint32 slot = slots[i];
        output[ slot ] = input[i];
    }
}


/// scatter a set of symbols into a sparse set of slots of a given output stream
///
template <typename input_iterator, typename slot_iterator, typename output_iterator>
struct device_scatter_dispatch
{
    static void enact(
        const uint32           n,
        const input_iterator   input,
        const slot_iterator    slots,
            output_iterator    output)
    {
        thrust::scatter(
            input,
            input + n,
            slots,
            output );
    }
};

/// scatter a set of symbols into a sparse set of slots of a given output stream
///
template <typename input_iterator, typename slot_iterator, typename storage_type, uint32 SYMBOL_SIZE, bool BIG_ENDIAN, typename index_type>
struct device_scatter_dispatch<
    input_iterator,
    slot_iterator,
    PackedStream<storage_type,uint8,SYMBOL_SIZE,BIG_ENDIAN,index_type> >
{
    typedef PackedStream<storage_type,uint8,SYMBOL_SIZE,BIG_ENDIAN,index_type> output_iterator;

    static void enact(
        const uint32           n,
        const input_iterator   input,
        const slot_iterator    slots,
            output_iterator    output)
    {
        // find out a set of ranges of input symbols covering distinct words in the output: this is done
        // looking at the words covered by each of the symbols, and reducing together all symbols falling
        // in the same word.
        thrust::device_vector<uint32> d_ranges( n );
        thrust::device_vector<uint32> d_keys( n );

        typedef typename PackedStream<storage_type,uint8,SYMBOL_SIZE,BIG_ENDIAN,index_type>::storage_type word_type;
        const uint32 SYMBOLS_PER_WORD = (8u*sizeof(word_type))/SYMBOL_SIZE;

        const uint32 n_ranges = uint32( thrust::reduce_by_key(
            thrust::make_transform_iterator( slots,     add_divide_functor( output.index(), SYMBOLS_PER_WORD ) ),
            thrust::make_transform_iterator( slots + n, add_divide_functor( output.index(), SYMBOLS_PER_WORD ) ),
            thrust::make_counting_iterator<uint32>(1u),
            d_keys.begin(),
            d_ranges.begin(),
            thrust::equal_to<uint32>(),
            thrust::maximum<uint32>() ).first - d_keys.begin() );

        const uint32 batch_size = cuda::max_grid_size();
        for (uint32 batch_begin = 0; batch_begin < n_ranges; batch_begin += batch_size)
        {
            const uint32 batch_end = nvbio::min( batch_begin + batch_size, n_ranges );

            // at this point we can scatter the identified ranges
            const uint32 blockdim = 128;
            const uint32 n_blocks = util::divide_ri( batch_end - batch_begin, blockdim );

            device_scatter_kernel<<<n_blocks,blockdim>>>(
                batch_begin,
                batch_end,
                d_ranges.begin(),
                input,
                slots,
                output );
        }
    }
};

/// scatter a set of symbols into a sparse set of slots of a given output stream
///
template <typename input_iterator, typename slot_iterator, typename output_iterator>
void device_scatter(
    const uint32            n,
    const input_iterator    input,
    const slot_iterator     slots,
          output_iterator   output)
{
    device_scatter_dispatch<input_iterator,slot_iterator,output_iterator>::enact(
        n,
        input,
        slots,
        output );
}

// ------------------------------------------------------------------------------------------------------------- //

/// pack a set of head flags into a bit-packed array
///
void pack_flags(
    const uint32    n,
    const uint8*    flags,
          uint32*   comp_flags);

/// build a set of head flags looking at adjacent keys
///
void build_head_flags(
    const uint32    n,
    const uint32*   keys,
          uint8*    flags);

/// build a set of head flags looking at adjacent keys
///
void build_head_flags(
    const uint32    n,
    const uint64*   keys,
          uint8*    flags);

} // namespace priv
} // namespace nvbio
